#!/usr/bin/env python3

# Generates the Markdown pages of the API reference.
# The script runs Doxygen with tools/docs/Doxyfile, then it converts the Doxygen XML to one
# Markdown page for each public header. MkDocs then makes the site from docs/ (see mkdocs.yml).
#
# Usage, from any directory: python3 tools/docs/generate_api_docs.py
# Doxygen 1.18 must be on the PATH.

import copy
import os
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

ROOT_DIR = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
DOXYFILE = os.path.join('tools', 'docs', 'Doxyfile')
DOXYGEN_OUTPUT_DIR = os.path.join('build', 'docs', 'doxygen')
API_OUTPUT_DIR = os.path.join('docs', 'api')
RTM_NAMESPACE_PREFIX = 'rtm::'

# Some public functions are in the shared headers of includes/rtm/impl/. Each of these functions goes on
# the pages of the public headers that it applies to (see find_pages_of_shared_function).
SHARED_HEADER_PAGES = {
	'mask_common.h': ['mask4f', 'mask4d', 'mask4i', 'mask4q'],
	'matrix_affine_common.h': ['matrix3x3f', 'matrix3x3d', 'matrix3x4f', 'matrix3x4d', 'matrix4x4f', 'matrix4x4d'],
	'matrix_cast.h': ['matrix3x3f', 'matrix3x3d', 'matrix3x4f', 'matrix3x4d', 'matrix4x4f', 'matrix4x4d'],
	'matrix_common.h': ['matrix3x3f', 'matrix3x3d', 'matrix3x4f', 'matrix3x4d', 'matrix4x4f', 'matrix4x4d'],
	'quat_common.h': ['quatf', 'quatd'],
	'qv_common.h': ['qvf', 'qvd'],
	'qvs_common.h': ['qvsf', 'qvsd'],
	'qvv_common.h': ['qvvf', 'qvvd'],
	'scalar_common.h': ['scalarf', 'scalard'],
	'vector_common.h': ['vector4f', 'vector4d'],
	'vq_common.h': ['vqf', 'vqd'],
	'vqm_common.h': ['vqmf', 'vqmd'],
	'vqs_common.h': ['vqsf', 'vqsd'],
	'vqv_common.h': ['vqvf', 'vqvd'],
}

# The public macros are in includes/rtm/impl/macros.*.impl.h. Client code includes rtm/macros.h.
MACRO_HEADER_PATTERN = re.compile(r'^includes/rtm/impl/macros\.\w+\.impl\.h$')

# RTM_DEFINE_FLOAT_CONSTANT makes a struct for each constant, and the struct converts to float and double.
EXTRA_COERCION_TYPES = {'float_constant': ['float', 'double']}

# The type names that end with the precision suffix: f = float32, d = float64.
PRECISION_TYPE_PATTERN = re.compile(r'^(vector4|quat|scalar|mask4|matrix\dx\d|vq|vqs|vqv|vqm|qv|qvs|qvv|float\d)([fd])$')

def run_doxygen():
	os.makedirs(os.path.join(ROOT_DIR, DOXYGEN_OUTPUT_DIR), exist_ok=True)

	# Doxyfile sets WARN_AS_ERROR: a public symbol without a comment stops Doxygen with an error
	result = subprocess.run(['doxygen', DOXYFILE], cwd=ROOT_DIR, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True)
	for line in result.stdout.splitlines():
		if line.strip():
			print(line)

	if result.returncode != 0:
		print('Doxygen failed with exit code {}'.format(result.returncode))
		sys.exit(result.returncode)

def read_headers():
	headers = {}
	for header_dir, _, filenames in os.walk(os.path.join(ROOT_DIR, 'includes', 'rtm')):
		for filename in sorted(filenames):
			if filename.endswith('.h'):
				path = os.path.join(header_dir, filename)
				with open(path) as f:
					headers[os.path.relpath(path, ROOT_DIR).replace(os.sep, '/')] = f.read()
	return headers

def find_coercion_types(headers):
	# Some functions return an rtm_impl struct with implicit conversion operators.
	# The pages show a conversion type in place of the struct name.
	# A template struct (for example matrix_caster) has one specialization for each type: the struct converts
	# to the types of all of its specializations.
	struct_pattern = re.compile(r'^\t\tstruct (\w+)(?:<[^>\n]*>)?\n\t\t\{\n(.*?)^\t\t\};', re.M | re.S)
	operator_pattern = re.compile(r'\boperator ([\w:]+)\(\)')

	coercion_types = dict(EXTRA_COERCION_TYPES)
	for content in headers.values():
		for match in struct_pattern.finditer(content):
			for type_name in operator_pattern.findall(match.group(2)):
				types = coercion_types.setdefault(match.group(1), [])
				if type_name not in types:
					types.append(type_name)
	return coercion_types

def find_opaque_types(headers):
	# A type with more than one definition in types.h has a different definition for each SIMD code path.
	# Doxygen sees only the scalar code path, thus the page does not show the definition of these types.
	names = re.findall(r'^\t(?:struct(?: alignas\(\d+\))?|using) (\w+)\b', headers['includes/rtm/types.h'], re.M)
	return {name for name in names if names.count(name) > 1}

def text_of(node):
	return '' if node is None else ''.join(node.itertext())

def escape_markdown(text):
	return text.replace('&', '&amp;').replace('<', '&lt;')

def slugify(text):
	# The same function as the toc extension of Python-Markdown
	text = re.sub(r'[^\w\s-]', '', text).strip().lower()
	return re.sub(r'[-\s]+', '-', text)

class DescriptionWriter:
	def __init__(self, links):
		self.links = links

	def write(self, node):
		return self.children(node).strip()

	def children(self, node):
		result = escape_markdown(node.text or '')
		for child in node:
			result += self.element(child)
			result += escape_markdown(child.tail or '')
		return result

	def element(self, node):
		tag = node.tag
		if tag == 'para':
			return '\n\n' + self.children(node).strip() + '\n\n'
		if tag == 'computeroutput':
			return '`' + text_of(node) + '`'
		if tag == 'emphasis':
			return '*' + self.children(node) + '*'
		if tag == 'bold':
			return '**' + self.children(node) + '**'
		if tag == 'ulink':
			return '[' + self.children(node) + '](' + node.get('url') + ')'
		if tag == 'ref':
			link = self.links.get(node.get('refid'))
			text = '`' + text_of(node) + '`'
			return '[' + text + '](' + link + ')' if link else text
		if tag == 'linebreak':
			return '<br>'
		if tag in ('itemizedlist', 'orderedlist'):
			marker = '-' if tag == 'itemizedlist' else '1.'
			items = [marker + ' ' + self.children(item).strip().replace('\n', '\n    ') for item in node.findall('listitem')]
			return '\n\n' + '\n'.join(items) + '\n\n'
		if tag == 'programlisting':
			lines = [self.code_line(line) for line in node.findall('codeline')]
			return '\n\n```cpp\n' + '\n'.join(lines) + '\n```\n\n'
		if tag == 'simplesect':
			title = {'return': 'Returns', 'note': 'Note', 'warning': 'Warning', 'see': 'See also'}.get(node.get('kind'), node.get('kind').capitalize())
			kind = 'warning' if node.get('kind') == 'warning' else 'note'
			return self.admonition(kind, title, self.children(node))
		if tag == 'xrefsect':
			title = node.findtext('xreftitle')
			kind = 'warning' if title == 'Deprecated' else 'note'
			return self.admonition(kind, title, self.children(node.find('xrefdescription')))
		if tag == 'parameterlist':
			items = []
			for item in node.findall('parameteritem'):
				names = ', '.join('`' + text_of(name) + '`' for name in item.iter('parametername'))
				items.append('- ' + names + ': ' + self.children(item.find('parameterdescription')).strip())
			return '\n\n**Parameters**\n\n' + '\n'.join(items) + '\n\n'
		return self.children(node)

	def code_line(self, node):
		text = ''
		for part in node.iter():
			if part.tag == 'sp':
				text += ' '
			elif part.text and part.tag != 'codeline':
				text += part.text
			if part.tag != 'codeline' and part.tail:
				text += part.tail
		return text

	def admonition(self, kind, title, body):
		body = re.sub(r'\n{3,}', '\n\n', body.strip())
		return '\n\n!!! {} "{}"\n\n'.format(kind, title) + '\n'.join('    ' + line if line else '' for line in body.split('\n')) + '\n\n'

def description_node_text(node):
	# The text of a description without the 'Deprecated' section that Doxygen adds for [[deprecated]]
	if node is None:
		return ''
	text = node.text or ''
	for child in node:
		if child.tag == 'para':
			text += (child.text or '') + ''.join(('' if part.tag == 'xrefsect' else text_of(part)) + (part.tail or '') for part in child)
		else:
			text += text_of(child)
		text += child.tail or ''
	return text

def has_description(node):
	return any(description_node_text(node.find(tag)).strip() for tag in ('briefdescription', 'detaileddescription'))

def page_name_of_header(header_file):
	# includes/rtm/packing/quatf.h -> packing_quatf
	return os.path.splitext(os.path.relpath(header_file, 'includes/rtm'))[0].replace('/', '_')

class Entry:
	def __init__(self, kind, name, node, location, section=None):
		self.kind = kind
		self.name = name
		self.node = node
		self.file = location.get('file')
		self.line = int(location.get('line'))
		self.section = section

def collect_entries(xml_dir):
	index = ET.parse(os.path.join(xml_dir, 'index.xml')).getroot()

	entries = []
	file_descriptions = {}
	for compound in index.findall('compound'):
		kind = compound.get('kind')
		name = compound.findtext('name')
		if kind not in ('namespace', 'struct', 'class', 'file'):
			continue
		if kind != 'file' and not (name == 'rtm' or name.startswith(RTM_NAMESPACE_PREFIX)):
			continue

		node = ET.parse(os.path.join(xml_dir, compound.get('refid') + '.xml')).getroot().find('compounddef')

		if kind == 'namespace':
			# The page of the header of a nested namespace shows the description of the namespace (for example rtm::constants)
			if name != 'rtm' and has_description(node):
				file_descriptions.setdefault(node.find('location').get('file'), node)

			# rtm::constants::pi -> constants::pi
			prefix = name[len(RTM_NAMESPACE_PREFIX):] + '::' if name != 'rtm' else ''
			for section in node.iter('sectiondef'):
				for member in section.findall('memberdef'):
					if member.get('kind') in ('function', 'typedef', 'enum', 'variable') and has_description(member):
						entries.append(Entry(member.get('kind'), prefix + member.findtext('name'), member, member.find('location'), section.findtext('header')))
		elif kind in ('struct', 'class'):
			if has_description(node):
				entries.append(Entry('struct', name[len(RTM_NAMESPACE_PREFIX):].replace(' ', ''), node, node.find('location')))
		else:
			location = node.find('location')
			if location is None:
				continue
			if has_description(node):
				file_descriptions[location.get('file')] = node
			for section in node.iter('sectiondef'):
				for member in section.findall('memberdef'):
					if member.get('kind') == 'define' and has_description(member):
						entries.append(Entry('define', member.findtext('name'), member, member.find('location')))

	entries.sort(key=lambda entry: (entry.file, entry.line))
	return entries, file_descriptions

class SignatureWriter:
	def __init__(self, coercion_types, opaque_types):
		self.coercion_types = coercion_types
		self.opaque_types = opaque_types

	def coercion_of(self, type_text):
		# rtm_impl::vector4f_vector_dot -> ['float']
		match = re.search(r'\brtm_impl::(\w+)', type_text)
		if not match:
			return None
		types = self.coercion_types.get(match.group(1))
		if types is None:
			for prefix, extra_types in EXTRA_COERCION_TYPES.items():
				if match.group(1).startswith(prefix):
					return extra_types
			print('Unknown return type: {}'.format(type_text))
			sys.exit(1)
		return types

	def clean(self, text):
		text = re.sub(r'\s+', ' ', text).strip()

		# vector4f_arg0 -> vector4f
		text = re.sub(r'\b(\w+)_arg\d+\b', r'\1', text)

		# Doxygen writes 'const float *input', RTM writes 'const float* input'
		text = re.sub(r'(\w) ([*&]+)(\w)', r'\1\2 \3', text)
		text = re.sub(r'(\w) ([*&]+)([,)])', r'\1\2\3', text)
		text = text.replace('< ', '<').replace(' >', '>')
		return text

	def return_type(self, type_text, page_type):
		# The type that the page shows for a function that returns an rtm_impl struct,
		# and the other types that the struct converts to.
		types = self.coercion_of(type_text)
		if types is None:
			return self.clean(type_text), []

		# A template conversion operator has a type name without the precision suffix, for example matrix3x3.
		# The page shows the type with the precision of the page, for example matrix3x3f.
		if page_type and page_type[-1] in 'fd':
			types = [candidate + page_type[-1] if PRECISION_TYPE_PATTERN.match(candidate + page_type[-1]) else candidate for candidate in types]

		chosen = page_type if page_type in types else types[0]
		return chosen, [candidate for candidate in types if candidate != chosen]

	def template_line(self, node):
		template_params = node.find('templateparamlist')
		if template_params is None:
			return ''
		params = []
		for param in template_params.findall('param'):
			param_text = text_of(param.find('type'))
			if param.findtext('declname'):
				param_text += ' ' + param.findtext('declname')
			if param.find('defval') is not None:
				param_text += ' = ' + text_of(param.find('defval'))
			params.append(self.clean(param_text))
		return 'template<' + ', '.join(params) + '>\n'

	def function(self, entry, page_type):
		node = entry.node
		raw_type = text_of(node.find('type'))
		return_type, other_types = self.return_type(raw_type, page_type)

		text = self.template_line(node)
		# A function that returns an rtm_impl struct is constexpr, but the conversion operator of the struct is not.
		# Thus the page does not show constexpr for these functions.
		if node.get('constexpr') == 'yes' and 'rtm_impl::' not in raw_type:
			text += 'constexpr '
		text += return_type + ' ' + entry.name + self.clean(node.findtext('argsstring'))
		return text, other_types

	def typedef(self, entry):
		if entry.name in self.opaque_types:
			return None
		return 'using {} = {};'.format(entry.name, self.clean(text_of(entry.node.find('type'))))

	def enum(self, entry):
		lines = ['enum class ' + entry.name if entry.node.get('strong') == 'yes' else 'enum ' + entry.name, '{']
		for value in entry.node.findall('enumvalue'):
			initializer = text_of(value.find('initializer')).strip()
			lines.append('\t' + value.findtext('name') + (' ' + initializer if initializer else '') + ',')
		lines.append('};')
		return '\n'.join(lines)

	def struct(self, entry):
		if entry.name in self.opaque_types:
			return None

		node = entry.node
		lines = [self.template_line(node).rstrip('\n')] if node.find('templateparamlist') is not None else []
		if '<' in entry.name and not lines:
			lines.append('template<>')
		lines += ['struct ' + entry.name.replace(',', ', '), '{']
		for section in node.findall('sectiondef'):
			for member in section.findall('memberdef'):
				comment = re.sub(r'\s+', ' ', description_node_text(member.find('briefdescription'))).strip()
				comment = ' // ' + comment if comment else ''
				if member.get('kind') == 'typedef':
					lines.append('\tusing {} = {};{}'.format(member.findtext('name'), self.clean(text_of(member.find('type'))), comment))
				elif member.get('kind') == 'variable':
					lines.append('\t{} {};{}'.format(self.clean(text_of(member.find('type'))), member.findtext('name'), comment))
		lines.append('};')
		return '\n'.join(line for line in lines if line)

	def define(self, entry):
		# A function-like macro shows its parameters, and an object-like macro shows its value
		if entry.node.find('param') is not None:
			params = [param.findtext('defname') for param in entry.node.findall('param')]
			return '#define {}({})'.format(entry.name, ', '.join(params))
		return '#define {} {}'.format(entry.name, text_of(entry.node.find('initializer')).strip())

def precision_of(text):
	# 'f' when the text names only float32 types, 'd' when it names only float64 types, otherwise None
	precisions = set()
	for token in re.findall(r'\w+', text):
		if token == 'float':
			precisions.add('f')
		elif token == 'double':
			precisions.add('d')
		else:
			match = PRECISION_TYPE_PATTERN.match(token)
			if match:
				precisions.add(match.group(2))
	return precisions.pop() if len(precisions) == 1 else None

def find_pages_of_shared_function(entry, pages, signature_writer):
	# A shared function goes on the page of each type that it returns.
	# The precision of its arguments selects between the float32 and the float64 page.
	node = entry.node
	raw_type = text_of(node.find('type'))
	coercion = signature_writer.coercion_of(raw_type)
	if coercion is not None:
		matches = [page for page in pages if any(page.startswith(candidate) for candidate in coercion)]
	else:
		return_type = signature_writer.clean(raw_type)
		matches = [page for page in pages if page == return_type]
	if matches:
		pages = matches

	precision = precision_of(signature_writer.clean(node.findtext('argsstring')))
	if precision:
		pages = [page for page in pages if page[-1] not in 'fd' or page[-1] == precision]
	return pages

def write_page(output_dir, page, include_path, entries, file_description, writer, signature_writer):
	lines = ['# {}'.format(include_path), '', '```cpp', '#include "{}"'.format(include_path), '```', '']
	if file_description is not None:
		for tag in ('briefdescription', 'detaileddescription'):
			text = writer.write(file_description.find(tag))
			if text:
				lines += [text, '']

	# Sections in the order of the header. Entries without a section go first.
	sections = []
	for entry in entries:
		if entry.section not in sections:
			sections.append(entry.section)
	sections.sort(key=lambda section: section is not None)

	for section in sections:
		if section:
			lines += ['## {}'.format(section), '']

		# The overloads of a function have one heading, at the position of the first overload
		names = []
		for entry in entries:
			if entry.section == section and entry.name not in names:
				names.append(entry.name)

		for name in names:
			heading = '`{}`'.format(name) if re.search(r'[^\w]', name) else name
			lines += ['### {}'.format(heading), '']

			# Consecutive overloads with the same description share one code block
			groups = []
			for entry in entries:
				if entry.section != section or entry.name != name:
					continue

				description = writer.write(entry.node.find('briefdescription')) + '\n\n' + writer.write(entry.node.find('detaileddescription'))
				if entry.kind == 'function':
					code, other_types = signature_writer.function(entry, page)
					if other_types:
						names = ['`{}`'.format(other) for other in other_types]
						names = names[0] if len(names) == 1 else ', '.join(names[:-1]) + ' and ' + names[-1]
						description += '\n\nThe return value also converts to {}.'.format(names)
				elif entry.kind == 'typedef':
					code = signature_writer.typedef(entry)
				elif entry.kind == 'enum':
					code = signature_writer.enum(entry)
				elif entry.kind == 'struct':
					code = signature_writer.struct(entry)
				elif entry.kind == 'define':
					code = signature_writer.define(entry)
				else:
					code = signature_writer.clean(text_of(entry.node.find('type'))) + ' ' + entry.name + ';'

				if code is None:
					description += '\n\nThe definition of this type depends on the SIMD code path. Use the functions of RTM to read and write its values.'
				description = re.sub(r'\n{3,}', '\n\n', description).strip()

				if groups and groups[-1][1] == description and code is not None and groups[-1][0]:
					groups[-1][0].append(code)
				else:
					groups.append(([code] if code is not None else [], description))

			for codes, description in groups:
				if codes:
					# A template line applies to one signature only, thus a blank line separates the signatures
					separator = '\n\n' if any('\n' in code for code in codes) else '\n'
					lines += ['```cpp', separator.join(codes), '```', '']
				lines += [description, '']

	with open(os.path.join(output_dir, page + '.md'), 'w') as f:
		f.write('\n'.join(lines))

def write_index(output_dir, pages):
	lines = ['# API reference', '', 'Each page lists the API of one public header.', '', '| Header | Entries |', '| --- | --- |']
	for page, include_path, count in sorted(pages, key=lambda page: page[1]):
		lines.append('| [`{}`]({}.md) | {} |'.format(include_path, page, count))
	lines.append('')

	with open(os.path.join(output_dir, 'index.md'), 'w') as f:
		f.write('\n'.join(lines))

def main():
	run_doxygen()

	headers = read_headers()
	signature_writer = SignatureWriter(find_coercion_types(headers), find_opaque_types(headers))
	entries, file_descriptions = collect_entries(os.path.join(ROOT_DIR, DOXYGEN_OUTPUT_DIR, 'xml'))

	# The page of each entry
	page_entries = {}
	include_paths = {}
	page_files = {}
	for entry in entries:
		if MACRO_HEADER_PATTERN.match(entry.file):
			pages = ['macros']
			include_paths['macros'] = 'rtm/macros.h'
		elif entry.file.startswith('includes/rtm/impl/'):
			shared_pages = SHARED_HEADER_PAGES.get(os.path.basename(entry.file))
			if shared_pages is None or entry.kind != 'function':
				print('No page for {} in {}: add the header to SHARED_HEADER_PAGES'.format(entry.name, entry.file))
				sys.exit(1)
			pages = find_pages_of_shared_function(entry, shared_pages, signature_writer)
		else:
			page = page_name_of_header(entry.file)
			pages = [page]
			include_paths[page] = entry.file[len('includes/'):]
			page_files[page] = entry.file

		for page in pages:
			page_entries.setdefault(page, []).append(entry)

	# The entries of the header of the page go first, then the shared functions.
	# A shared function goes in the section of the function with the same name in the header of the page,
	# thus all of the overloads have one heading. Entry objects are shared between pages, thus each page
	# gets copies of the shared entries.
	for page in page_entries:
		own_sections = {}
		for entry in page_entries[page]:
			if entry.file == page_files.get(page):
				own_sections.setdefault(entry.name, entry.section)

		entries = []
		for entry in page_entries[page]:
			if entry.file != page_files.get(page) and entry.name in own_sections:
				entry = copy.copy(entry)
				entry.section = own_sections[entry.name]
			entries.append(entry)

		page_entries[page] = sorted(entries, key=lambda entry, page_file=page_files.get(page): (entry.file != page_file, entry.file, entry.line))

	# Each entry links to its heading. A shared function links to its first page.
	links = {}
	for page, page_entry_list in sorted(page_entries.items()):
		for entry in page_entry_list:
			links.setdefault(entry.node.get('id'), '{}.md#{}'.format(page, slugify(entry.name)))

	# The output folder holds generated pages only
	output_dir = os.path.join(ROOT_DIR, API_OUTPUT_DIR)
	if os.path.exists(output_dir):
		shutil.rmtree(output_dir)
	os.makedirs(output_dir)

	writer = DescriptionWriter(links)
	pages = []
	for page, page_entry_list in sorted(page_entries.items()):
		write_page(output_dir, page, include_paths[page], page_entry_list, file_descriptions.get(page_files.get(page)), writer, signature_writer)
		pages.append((page, include_paths[page], len(page_entry_list)))

	write_index(output_dir, pages)
	print('Wrote {} pages in {}'.format(len(pages) + 1, API_OUTPUT_DIR))

if __name__ == '__main__':
	main()
