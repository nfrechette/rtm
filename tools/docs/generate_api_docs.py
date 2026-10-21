#!/usr/bin/env python3

# Generates the Markdown pages of the API reference.
# The script runs Doxygen with tools/docs/Doxyfile, then it converts the Doxygen XML to one
# Markdown page for each public header. MkDocs then makes the site from docs/ (see mkdocs.yml).
#
# Usage, from any directory: python3 tools/docs/generate_api_docs.py

import argparse
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

def parse_argv():
	parser = argparse.ArgumentParser(description='Generates the Markdown pages of the RTM API reference.')
	parser.add_argument('--doxygen', default='doxygen', help='The Doxygen executable to run')
	return parser.parse_args()

def run_doxygen(doxygen):
	os.makedirs(os.path.join(ROOT_DIR, DOXYGEN_OUTPUT_DIR), exist_ok=True)

	result = subprocess.run([doxygen, DOXYFILE], cwd=ROOT_DIR, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True)
	warnings = [line for line in result.stdout.splitlines() if line.strip()]
	for line in warnings:
		print(line)

	if result.returncode != 0:
		print('Doxygen failed with exit code {}'.format(result.returncode))
		sys.exit(result.returncode)

	print('Doxygen found {} warnings'.format(len(warnings)))

def find_coercion_types():
	# Some functions return an rtm_impl struct with implicit conversion operators.
	# The pages show the conversion types in place of the struct name.
	struct_pattern = re.compile(r'^\t\tstruct (\w+)\n\t\t\{\n(.*?)^\t\t\};', re.M | re.S)
	operator_pattern = re.compile(r'\boperator ([\w:]+)\(\)')

	# Some of these structs are in the headers of includes/rtm/impl/.
	coercion_types = {}
	for header_dir, _, filenames in os.walk(os.path.join(ROOT_DIR, 'includes', 'rtm')):
		for filename in sorted(filenames):
			if not filename.endswith('.h'):
				continue

			with open(os.path.join(header_dir, filename)) as f:
				content = f.read()

			for match in struct_pattern.finditer(content):
				types = operator_pattern.findall(match.group(2))
				if types:
					coercion_types[match.group(1)] = types

	return coercion_types

def clean_type(text, coercion_types):
	text = re.sub(r'\s+', ' ', text).strip()

	# rtm_impl::vector4f_vector_dot -> float
	def replace_coercion(match):
		types = coercion_types.get(match.group(1))
		if not types:
			print('Unknown return type: {}'.format(match.group(0)))
			sys.exit(1)
		return types[0]
	text = re.sub(r'\brtm_impl::(\w+)(?:\s*<[^>]*>)?', replace_coercion, text)

	# vector4f_arg0 -> vector4f
	text = re.sub(r'\b(\w+)_arg\d+\b', r'\1', text)

	# Doxygen writes 'const float *input', RTM writes 'const float* input'
	text = re.sub(r'(\w) ([*&]+)(\w)', r'\1\2 \3', text)
	text = re.sub(r'(\w) ([*&]+)([,)])', r'\1\2\3', text)
	text = text.replace('< ', '<').replace(' >', '>')
	return text

def escape_markdown(text):
	return text.replace('&', '&amp;').replace('<', '&lt;')

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
			return '`' + ''.join(node.itertext()) + '`'
		if tag == 'emphasis':
			return '*' + self.children(node) + '*'
		if tag == 'bold':
			return '**' + self.children(node) + '**'
		if tag == 'ulink':
			return '[' + self.children(node) + '](' + node.get('url') + ')'
		if tag == 'ref':
			link = self.links.get(node.get('refid'))
			text = '`' + ''.join(node.itertext()) + '`'
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
				names = ', '.join('`' + ''.join(name.itertext()) + '`' for name in item.iter('parametername'))
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

def has_description(member):
	# Doxygen adds a 'Deprecated' section for [[deprecated]]. A member with only this section has no comment.
	for tag in ('briefdescription', 'detaileddescription'):
		node = member.find(tag)
		text = (node.text or '') + ''.join(''.join(child.itertext()) + (child.tail or '') for child in node if child.tag != 'para')
		for para in node.findall('para'):
			text += (para.text or '') + ''.join(('' if child.tag == 'xrefsect' else ''.join(child.itertext())) + (child.tail or '') for child in para)
		if text.strip():
			return True
	return False

def page_name(header_file):
	# includes/rtm/packing/quatf.h -> packing_quatf
	path = os.path.splitext(os.path.relpath(header_file, os.path.join('includes', 'rtm')))[0]
	return path.replace(os.sep, '_').replace('/', '_')

def collect_members(xml_dir):
	# All the public API is in namespace rtm. Each member has its header file and its line.
	namespace = ET.parse(os.path.join(xml_dir, 'namespacertm.xml')).getroot()

	members = []
	for section in namespace.iter('sectiondef'):
		section_name = section.findtext('header')
		for member in section.findall('memberdef'):
			if member.get('kind') != 'function' or not has_description(member):
				continue

			location = member.find('location')
			members.append({
				'node': member,
				'name': member.findtext('name'),
				'file': location.get('file'),
				'line': int(location.get('line')),
				'section': section_name,
			})

	members.sort(key=lambda member: (member['file'], member['line']))
	return members

def signature(member, coercion_types):
	node = member['node']
	text = ''

	template_params = node.find('templateparamlist')
	if template_params is not None:
		params = []
		for param in template_params.findall('param'):
			param_text = ''.join(param.find('type').itertext())
			if param.findtext('declname'):
				param_text += ' ' + param.findtext('declname')
			if param.find('defval') is not None:
				param_text += ' = ' + ''.join(param.find('defval').itertext())
			params.append(param_text)
		text += 'template<' + ', '.join(params) + '>\n'

	# A function that returns an rtm_impl struct is constexpr, but the conversion operator of the struct is not.
	# Thus the page does not show constexpr for these functions.
	return_type = ''.join(node.find('type').itertext())
	if node.get('constexpr') == 'yes' and 'rtm_impl::' not in return_type:
		text += 'constexpr '

	text += clean_type(return_type, coercion_types) + ' '
	text += member['name'] + clean_type(node.findtext('argsstring'), coercion_types)
	return text

def write_page(output_dir, header_file, members, writer, coercion_types):
	include_path = os.path.relpath(header_file, 'includes').replace(os.sep, '/')
	lines = ['# {}'.format(include_path), '', '```cpp', '#include "{}"'.format(include_path), '```', '']

	# Sections in the order of the header. Members without a section go first.
	sections = []
	for member in members:
		if member['section'] not in sections:
			sections.append(member['section'])
	sections.sort(key=lambda section: section is not None)

	for section in sections:
		if section:
			lines += ['## {}'.format(section), '']

		# The overloads of a function have one heading, at the position of the first overload.
		names = []
		for member in members:
			if member['section'] == section and member['name'] not in names:
				names.append(member['name'])

		for name in names:
			lines += ['### {}'.format(name), '']

			# Consecutive overloads with the same description share one code block.
			groups = []
			for member in members:
				if member['section'] != section or member['name'] != name:
					continue
				description = writer.write(member['node'].find('briefdescription')) + '\n\n' + writer.write(member['node'].find('detaileddescription'))
				description = re.sub(r'\n{3,}', '\n\n', description).strip()
				if groups and groups[-1][1] == description:
					groups[-1][0].append(signature(member, coercion_types))
				else:
					groups.append(([signature(member, coercion_types)], description))

			for signatures, description in groups:
				# A template line applies to one signature only, thus a blank line separates the signatures.
				separator = '\n\n' if any('\n' in text for text in signatures) else '\n'
				lines += ['```cpp', separator.join(signatures), '```', '', description, '']

	with open(os.path.join(output_dir, page_name(header_file) + '.md'), 'w') as f:
		f.write('\n'.join(lines))

def write_index(output_dir, pages):
	lines = ['# API reference', '', 'Each page lists the API of one public header.', '', '| Header | Functions |', '| --- | --- |']
	for header_file, count in pages:
		include_path = os.path.relpath(header_file, 'includes').replace(os.sep, '/')
		lines.append('| [`{}`]({}.md) | {} |'.format(include_path, page_name(header_file), count))
	lines.append('')

	with open(os.path.join(output_dir, 'index.md'), 'w') as f:
		f.write('\n'.join(lines))

def main():
	args = parse_argv()

	run_doxygen(args.doxygen)

	coercion_types = find_coercion_types()
	members = collect_members(os.path.join(ROOT_DIR, DOXYGEN_OUTPUT_DIR, 'xml'))

	headers = []
	for member in members:
		if member['file'] not in headers:
			headers.append(member['file'])

	# Each overload links to the heading of its function.
	links = {}
	for member in members:
		links[member['node'].get('id')] = '{}.md#{}'.format(page_name(member['file']), member['name'].lower())

	# The output folder holds generated pages only.
	output_dir = os.path.join(ROOT_DIR, API_OUTPUT_DIR)
	if os.path.exists(output_dir):
		shutil.rmtree(output_dir)
	os.makedirs(output_dir)

	writer = DescriptionWriter(links)
	pages = []
	for header_file in headers:
		header_members = [member for member in members if member['file'] == header_file]
		write_page(output_dir, header_file, header_members, writer, coercion_types)
		pages.append((header_file, len(header_members)))

	write_index(output_dir, pages)
	print('Wrote {} pages in {}'.format(len(pages) + 1, API_OUTPUT_DIR))

if __name__ == '__main__':
	main()
