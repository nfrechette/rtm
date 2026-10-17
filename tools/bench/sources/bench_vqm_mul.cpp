////////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
// Copyright (c) 2026 Nicholas Frechette & Realtime Math contributors
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
////////////////////////////////////////////////////////////////////////////////

#include <benchmark/benchmark.h>

#include <rtm/experimental/vqmf.h>
#include <rtm/experimental/vqmd.h>

#include <cstddef>
#include <random>
#include <string>

using namespace rtm;

// Each variant has the same fast paths as vqm_mul and vqm_inverse: the flag in the m22 slot.
// Only the full path changes. The variants are templates for vqmf and vqmd.
//     - ref: the library function, which builds the rotation matrix once from the quaternion and uses matrix multiplications
//     - quat: the function rotates each row (or column) of a matrix with quat_mul_vector3, as the library did before
//     - soa_cross: the function rotates the three rows (or columns) of a matrix in one pass, in structure of arrays form,
//       with the cross product formula
//     - soa_quat: as soa_cross, but with the two quaternion products q * v * q^-1, as quat_mul_vector3 computes them
//     - soa_quat_split: as soa_quat, but vqm_mul computes (M1 * R1) * (M2 * R1^T) as ref does, not M1 * (R1 * M2 * R1^T).
//       M1 * R1 does not depend on rhs, thus a chain of products has a shorter dependency path

//////////////////////////////////////////////////////////////////////////
// Rotates three vectors with a quaternion: v' = v * R.
// The vectors are in structure of arrays form: x, y, and z hold the components, one vector in each lane.
// v' = v + w * t + cross(q.xyz, t) with t = 2 * cross(q.xyz, v)
//////////////////////////////////////////////////////////////////////////
template<typename QuatType, typename Vector4Type>
RTM_FORCE_INLINE void RTM_SIMD_CALL quat_mul_vector3_soa_cross(const QuatType& rotation, Vector4Type& x, Vector4Type& y, Vector4Type& z) RTM_NO_EXCEPT
{
	const Vector4Type q = quat_to_vector(rotation);
	const Vector4Type qx = vector_dup_x(q);
	const Vector4Type qy = vector_dup_y(q);
	const Vector4Type qz = vector_dup_z(q);
	const Vector4Type qw = vector_dup_w(q);

	// t = 2 * cross(q.xyz, v)
	Vector4Type tx = vector_neg_mul_sub(qz, y, vector_mul(qy, z));
	Vector4Type ty = vector_neg_mul_sub(qx, z, vector_mul(qz, x));
	Vector4Type tz = vector_neg_mul_sub(qy, x, vector_mul(qx, y));
	tx = vector_add(tx, tx);
	ty = vector_add(ty, ty);
	tz = vector_add(tz, tz);

	// v' = v + w * t + cross(q.xyz, t)
	const Vector4Type rx = vector_add(vector_mul_add(qw, tx, x), vector_neg_mul_sub(qz, ty, vector_mul(qy, tz)));
	const Vector4Type ry = vector_add(vector_mul_add(qw, ty, y), vector_neg_mul_sub(qx, tz, vector_mul(qz, tx)));
	const Vector4Type rz = vector_add(vector_mul_add(qw, tz, z), vector_neg_mul_sub(qy, tx, vector_mul(qx, ty)));

	x = rx;
	y = ry;
	z = rz;
}

//////////////////////////////////////////////////////////////////////////
// Rotates three vectors with a quaternion: v' = v * R.
// The vectors are in structure of arrays form: x, y, and z hold the components, one vector in each lane.
// The function computes the two quaternion products q * v * q^-1 (Hamilton order) with v a pure quaternion.
//////////////////////////////////////////////////////////////////////////
template<typename QuatType, typename Vector4Type>
RTM_FORCE_INLINE void RTM_SIMD_CALL quat_mul_vector3_soa_quat(const QuatType& rotation, Vector4Type& x, Vector4Type& y, Vector4Type& z) RTM_NO_EXCEPT
{
	const Vector4Type q = quat_to_vector(rotation);
	const Vector4Type qx = vector_dup_x(q);
	const Vector4Type qy = vector_dup_y(q);
	const Vector4Type qz = vector_dup_z(q);
	const Vector4Type qw = vector_dup_w(q);

	// t = q * v
	const Vector4Type tw = vector_neg(vector_mul_add(qz, z, vector_mul_add(qy, y, vector_mul(qx, x))));
	const Vector4Type tx = vector_neg_mul_sub(qz, y, vector_mul_add(qy, z, vector_mul(qw, x)));
	const Vector4Type ty = vector_neg_mul_sub(qx, z, vector_mul_add(qz, x, vector_mul(qw, y)));
	const Vector4Type tz = vector_neg_mul_sub(qy, x, vector_mul_add(qx, y, vector_mul(qw, z)));

	// v' = t * q^-1, the [w] part is zero
	x = vector_neg_mul_sub(ty, qz, vector_mul_add(tz, qy, vector_neg_mul_sub(tw, qx, vector_mul(tx, qw))));
	y = vector_neg_mul_sub(tz, qx, vector_mul_add(tx, qz, vector_neg_mul_sub(tw, qy, vector_mul(ty, qw))));
	z = vector_neg_mul_sub(tx, qy, vector_mul_add(ty, qx, vector_neg_mul_sub(tw, qz, vector_mul(tz, qw))));
}

//////////////////////////////////////////////////////////////////////////
// Rotates three vectors in structure of arrays form with one of the two formulas.
//////////////////////////////////////////////////////////////////////////
template<bool use_quat_products, typename QuatType, typename Vector4Type>
RTM_FORCE_INLINE void RTM_SIMD_CALL quat_mul_vector3_soa(const QuatType& rotation, Vector4Type& x, Vector4Type& y, Vector4Type& z) RTM_NO_EXCEPT
{
	if (use_quat_products)
		quat_mul_vector3_soa_quat(rotation, x, y, z);
	else
		quat_mul_vector3_soa_cross(rotation, x, y, z);
}

//////////////////////////////////////////////////////////////////////////
// Returns R^T * M * R where R is the rotation matrix of the quaternion.
// The function rotates the rows of M with the quaternion (M * R), then the columns of the result (R^T * X).
// A rotation of the column vectors with the quaternion gives R^T * X because (X^T * R)^T = R^T * X.
//////////////////////////////////////////////////////////////////////////
template<bool use_quat_products, typename Matrix3x3Type, typename QuatType>
RTM_FORCE_INLINE Matrix3x3Type RTM_SIMD_CALL matrix_rotate_soa(const Matrix3x3Type& input, const QuatType& rotation) RTM_NO_EXCEPT
{
	// The columns of M hold the components of its rows
	Matrix3x3Type result = matrix_transpose(input);
	quat_mul_vector3_soa<use_quat_products>(rotation, result.x_axis, result.y_axis, result.z_axis);

	// The rows of X = M * R hold the components of its columns
	result = matrix_transpose(result);
	quat_mul_vector3_soa<use_quat_products>(rotation, result.x_axis, result.y_axis, result.z_axis);

	return result;
}

//////////////////////////////////////////////////////////////////////////
// The fast path of vqm_mul: the scale/shear part of rhs is the identity.
//////////////////////////////////////////////////////////////////////////
template<typename TransformType, typename QuatType>
RTM_FORCE_INLINE TransformType RTM_SIMD_CALL vqm_mul_rhs_identity(const TransformType& lhs, const TransformType& rhs, const QuatType& rotation) RTM_NO_EXCEPT
{
	// M2 = I, thus v3 = v1 * R2 + v2 and M3 = M1
	TransformType result;
	result.rotation = rotation;
	result.translation_m22 = vector_mix<mix4::x, mix4::y, mix4::z, mix4::d>(vector_add(quat_mul_vector3(lhs.translation_m22, rhs.rotation), rhs.translation_m22), lhs.translation_m22);
	result.x_axis_m20 = lhs.x_axis_m20;
	result.y_axis_m21 = lhs.y_axis_m21;
	return result;
}

template<typename TransformType>
RTM_FORCE_NOINLINE TransformType RTM_SIMD_CALL vqm_mul_ref(const TransformType& lhs, const TransformType& rhs) RTM_NO_EXCEPT
{
	return vqm_mul(lhs, rhs);
}

template<typename TransformType>
RTM_FORCE_NOINLINE TransformType RTM_SIMD_CALL vqm_mul_quat(const TransformType& lhs, const TransformType& rhs) RTM_NO_EXCEPT
{
	using QuatType = typename related_types<TransformType>::quat;
	using Vector4Type = typename related_types<TransformType>::vector4;
	using Matrix3x3Type = typename related_types<TransformType>::matrix3x3;

	const QuatType rotation = quat_mul(lhs.rotation, rhs.rotation);

	if (rtm_impl::vqm_has_identity_scale_shear(rhs))
		return vqm_mul_rhs_identity(lhs, rhs, rotation);

	const QuatType inv_lhs_rotation = quat_conjugate(lhs.rotation);

	Matrix3x3Type lhs_scale_shear = vqm_get_scale_shear(lhs);
	Matrix3x3Type rhs_scale_shear = rtm_impl::vqm_get_scale_shear_no_flag(rhs);

	const Vector4Type translation = vector_add(quat_mul_vector3(matrix_mul_vector3(lhs.translation_m22, rhs_scale_shear), rhs.rotation), rhs.translation_m22);

	// M3 = (M1 * R1) * (M2 * R1^T)
	rhs_scale_shear.x_axis = quat_mul_vector3(rhs_scale_shear.x_axis, inv_lhs_rotation);
	rhs_scale_shear.y_axis = quat_mul_vector3(rhs_scale_shear.y_axis, inv_lhs_rotation);
	rhs_scale_shear.z_axis = quat_mul_vector3(rhs_scale_shear.z_axis, inv_lhs_rotation);

	lhs_scale_shear.x_axis = quat_mul_vector3(lhs_scale_shear.x_axis, lhs.rotation);
	lhs_scale_shear.y_axis = quat_mul_vector3(lhs_scale_shear.y_axis, lhs.rotation);
	lhs_scale_shear.z_axis = quat_mul_vector3(lhs_scale_shear.z_axis, lhs.rotation);

	const Matrix3x3Type scale_shear = matrix_mul(lhs_scale_shear, rhs_scale_shear);
	return rtm_impl::vqm_set_no_flag(rotation, translation, scale_shear);
}

template<typename TransformType, bool use_quat_products>
RTM_FORCE_NOINLINE TransformType RTM_SIMD_CALL vqm_mul_soa(const TransformType& lhs, const TransformType& rhs) RTM_NO_EXCEPT
{
	using QuatType = typename related_types<TransformType>::quat;
	using Vector4Type = typename related_types<TransformType>::vector4;
	using Matrix3x3Type = typename related_types<TransformType>::matrix3x3;

	const QuatType rotation = quat_mul(lhs.rotation, rhs.rotation);

	if (rtm_impl::vqm_has_identity_scale_shear(rhs))
		return vqm_mul_rhs_identity(lhs, rhs, rotation);

	const Matrix3x3Type lhs_scale_shear = vqm_get_scale_shear(lhs);
	const Matrix3x3Type rhs_scale_shear = rtm_impl::vqm_get_scale_shear_no_flag(rhs);
	const Vector4Type translation = vector_add(quat_mul_vector3(matrix_mul_vector3(lhs.translation_m22, rhs_scale_shear), rhs.rotation), rhs.translation_m22);

	// M3 = M1 * (R1 * M2 * R1^T) and R1 is the transpose of the rotation matrix of conjugate(q1)
	const Matrix3x3Type rhs_rotated = matrix_rotate_soa<use_quat_products>(rhs_scale_shear, quat_conjugate(lhs.rotation));
	const Matrix3x3Type scale_shear = matrix_mul(lhs_scale_shear, rhs_rotated);

	return rtm_impl::vqm_set_no_flag(rotation, translation, scale_shear);
}

template<typename TransformType>
RTM_FORCE_NOINLINE TransformType RTM_SIMD_CALL vqm_mul_soa_split(const TransformType& lhs, const TransformType& rhs) RTM_NO_EXCEPT
{
	using QuatType = typename related_types<TransformType>::quat;
	using Vector4Type = typename related_types<TransformType>::vector4;
	using Matrix3x3Type = typename related_types<TransformType>::matrix3x3;

	const QuatType rotation = quat_mul(lhs.rotation, rhs.rotation);

	if (rtm_impl::vqm_has_identity_scale_shear(rhs))
		return vqm_mul_rhs_identity(lhs, rhs, rotation);

	const Matrix3x3Type lhs_scale_shear = vqm_get_scale_shear(lhs);
	const Matrix3x3Type rhs_scale_shear = rtm_impl::vqm_get_scale_shear_no_flag(rhs);
	const Vector4Type translation = vector_add(quat_mul_vector3(matrix_mul_vector3(lhs.translation_m22, rhs_scale_shear), rhs.rotation), rhs.translation_m22);

	// M3 = (M1 * R1) * (M2 * R1^T)
	// The columns of a matrix hold the components of its rows
	Matrix3x3Type lhs_rotated = matrix_transpose(lhs_scale_shear);
	quat_mul_vector3_soa_quat(lhs.rotation, lhs_rotated.x_axis, lhs_rotated.y_axis, lhs_rotated.z_axis);
	lhs_rotated = matrix_transpose(lhs_rotated);

	Matrix3x3Type rhs_rotated = matrix_transpose(rhs_scale_shear);
	quat_mul_vector3_soa_quat(quat_conjugate(lhs.rotation), rhs_rotated.x_axis, rhs_rotated.y_axis, rhs_rotated.z_axis);
	rhs_rotated = matrix_transpose(rhs_rotated);

	const Matrix3x3Type scale_shear = matrix_mul(lhs_rotated, rhs_rotated);

	return rtm_impl::vqm_set_no_flag(rotation, translation, scale_shear);
}

template<typename TransformType>
RTM_FORCE_NOINLINE TransformType RTM_SIMD_CALL vqm_inverse_ref(const TransformType& input) RTM_NO_EXCEPT
{
	return vqm_inverse(input);
}

template<typename TransformType>
RTM_FORCE_NOINLINE TransformType RTM_SIMD_CALL vqm_inverse_quat(const TransformType& input) RTM_NO_EXCEPT
{
	using QuatType = typename related_types<TransformType>::quat;
	using Vector4Type = typename related_types<TransformType>::vector4;
	using Matrix3x3Type = typename related_types<TransformType>::matrix3x3;

	if (rtm_impl::vqm_has_identity_scale_shear(input))
		return rtm_impl::vqm_inverse_identity_scale_shear(input);

	const Matrix3x3Type inv_scale_shear = matrix_inverse(rtm_impl::vqm_get_scale_shear_no_flag(input));
	const QuatType inv_rotation = quat_conjugate(input.rotation);
	const Vector4Type inv_translation = matrix_mul_vector3(quat_mul_vector3(vector_neg(input.translation_m22), inv_rotation), inv_scale_shear);

	// M' = R^T * (M^-1 * R)
	// M^-1 * R rotates each row of M^-1 with q, and the rotation of the identity rows with conjugate(q) gives R^T
	Matrix3x3Type inv_rotated_scale_shear;
	inv_rotated_scale_shear.x_axis = quat_mul_vector3(inv_scale_shear.x_axis, input.rotation);
	inv_rotated_scale_shear.y_axis = quat_mul_vector3(inv_scale_shear.y_axis, input.rotation);
	inv_rotated_scale_shear.z_axis = quat_mul_vector3(inv_scale_shear.z_axis, input.rotation);

	Matrix3x3Type inv_rotation_mtx = matrix_identity();
	inv_rotation_mtx.x_axis = quat_mul_vector3(inv_rotation_mtx.x_axis, inv_rotation);
	inv_rotation_mtx.y_axis = quat_mul_vector3(inv_rotation_mtx.y_axis, inv_rotation);
	inv_rotation_mtx.z_axis = quat_mul_vector3(inv_rotation_mtx.z_axis, inv_rotation);

	const Matrix3x3Type scale_shear = matrix_mul(inv_rotation_mtx, inv_rotated_scale_shear);
	return rtm_impl::vqm_set_no_flag(inv_rotation, inv_translation, scale_shear);
}

template<typename TransformType, bool use_quat_products>
RTM_FORCE_NOINLINE TransformType RTM_SIMD_CALL vqm_inverse_soa(const TransformType& input) RTM_NO_EXCEPT
{
	using QuatType = typename related_types<TransformType>::quat;
	using Vector4Type = typename related_types<TransformType>::vector4;
	using Matrix3x3Type = typename related_types<TransformType>::matrix3x3;

	if (rtm_impl::vqm_has_identity_scale_shear(input))
		return rtm_impl::vqm_inverse_identity_scale_shear(input);

	const Matrix3x3Type inv_scale_shear = matrix_inverse(rtm_impl::vqm_get_scale_shear_no_flag(input));
	const QuatType inv_rotation = quat_conjugate(input.rotation);
	const Vector4Type inv_translation = matrix_mul_vector3(quat_mul_vector3(vector_neg(input.translation_m22), inv_rotation), inv_scale_shear);

	// M' = R^T * M^-1 * R
	const Matrix3x3Type scale_shear = matrix_rotate_soa<use_quat_products>(inv_scale_shear, input.rotation);

	return rtm_impl::vqm_set_no_flag(inv_rotation, inv_translation, scale_shear);
}

template<typename TransformType>
RTM_FORCE_NOINLINE typename related_types<TransformType>::vector4 RTM_SIMD_CALL vqm_mul_point3_ref(const typename related_types<TransformType>::vector4& point, const TransformType& transform) RTM_NO_EXCEPT
{
	return vqm_mul_point3(point, transform);
}

// Each set of inputs has this number of transforms: 256 vqmd transforms use 32 KB
constexpr size_t k_num_transforms = 256;

// The latency loop multiplies chains of this length, as a hierarchy with this depth does
constexpr size_t k_chain_length = 8;

enum class input_kind
{
	rigid,		// All the transforms have the identity scale/shear part, with the flag
	general,	// All the transforms have a scale/shear part
	mixed,		// Half of the transforms have the identity scale/shear part, in random order
};

static const char* get_input_kind_name(input_kind kind)
{
	switch (kind)
	{
	case input_kind::rigid:		return "rigid";
	case input_kind::general:	return "general";
	default:					return "mixed";
	}
}

template<typename TransformType>
struct bench_inputs
{
	using Vector4Type = typename related_types<TransformType>::vector4;

	TransformType lhs[k_num_transforms];
	TransformType rhs[k_num_transforms];
	TransformType results[k_num_transforms];
	Vector4Type points[k_num_transforms];
	Vector4Type point_results[k_num_transforms];
};

template<typename TransformType>
static TransformType make_random_transform(std::mt19937& generator, bool is_rigid)
{
	using FloatType = typename related_types<TransformType>::float1;
	using QuatType = typename related_types<TransformType>::quat;
	using Vector4Type = typename related_types<TransformType>::vector4;
	using Matrix3x3Type = typename related_types<TransformType>::matrix3x3;

	std::uniform_real_distribution<double> angle_distribution(-3.14159, 3.14159);
	std::uniform_real_distribution<double> translation_distribution(-10.0, 10.0);
	std::uniform_real_distribution<double> scale_distribution(0.9, 1.1);
	std::uniform_real_distribution<double> shear_distribution(-0.1, 0.1);

	const QuatType rotation = quat_normalize(quat_from_euler(FloatType(angle_distribution(generator)), FloatType(angle_distribution(generator)), FloatType(angle_distribution(generator))));
	const Vector4Type translation = vector_set(FloatType(translation_distribution(generator)), FloatType(translation_distribution(generator)), FloatType(translation_distribution(generator)));
	if (is_rigid)
		return vqm_set(rotation, translation, vector_set(FloatType(1.0)));

	// The scale is near 1.0: the product of a chain of transforms does not overflow or underflow
	const Matrix3x3Type scale_shear = matrix_set(
		vector_set(FloatType(scale_distribution(generator)), FloatType(shear_distribution(generator)), FloatType(shear_distribution(generator))),
		vector_set(FloatType(shear_distribution(generator)), FloatType(scale_distribution(generator)), FloatType(shear_distribution(generator))),
		vector_set(FloatType(shear_distribution(generator)), FloatType(shear_distribution(generator)), FloatType(scale_distribution(generator))));
	return vqm_set(rotation, translation, scale_shear);
}

template<typename TransformType>
static bench_inputs<TransformType>& get_inputs(input_kind kind)
{
	using FloatType = typename related_types<TransformType>::float1;

	// Static storage gives the alignment of the transforms
	static bench_inputs<TransformType> inputs[3];
	static bool is_initialized = false;

	if (!is_initialized)
	{
		// A fixed seed gives the same inputs for each run and each variant
		std::mt19937 generator(0x5EED);
		std::bernoulli_distribution is_rigid_distribution(0.5);
		std::uniform_real_distribution<double> point_distribution(-10.0, 10.0);

		for (int kind_index = 0; kind_index < 3; ++kind_index)
		{
			bench_inputs<TransformType>& kind_inputs = inputs[kind_index];
			for (size_t index = 0; index < k_num_transforms; ++index)
			{
				const bool is_lhs_rigid = kind_index == int(input_kind::rigid) || (kind_index == int(input_kind::mixed) && is_rigid_distribution(generator));
				const bool is_rhs_rigid = kind_index == int(input_kind::rigid) || (kind_index == int(input_kind::mixed) && is_rigid_distribution(generator));
				kind_inputs.lhs[index] = make_random_transform<TransformType>(generator, is_lhs_rigid);
				kind_inputs.rhs[index] = make_random_transform<TransformType>(generator, is_rhs_rigid);
				kind_inputs.points[index] = vector_set(FloatType(point_distribution(generator)), FloatType(point_distribution(generator)), FloatType(point_distribution(generator)));
			}
		}

		is_initialized = true;
	}

	return inputs[int(kind)];
}

// The independent products of two arrays of transforms
template<typename TransformType, TransformType (RTM_SIMD_CALL *mul_fn)(const TransformType&, const TransformType&)>
static void bm_vqm_mul_throughput(benchmark::State& state, input_kind kind)
{
	bench_inputs<TransformType>& inputs = get_inputs<TransformType>(kind);

	for (auto _ : state)
	{
		for (size_t index = 0; index < k_num_transforms; ++index)
			inputs.results[index] = mul_fn(inputs.lhs[index], inputs.rhs[index]);

		benchmark::ClobberMemory();
	}

	state.SetItemsProcessed(int64_t(state.iterations()) * int64_t(k_num_transforms));
}

// Chains of products: each result is the input rhs of the next product, as in a hierarchy (local_to_world = vqm_mul(local_to_parent, parent_to_world))
template<typename TransformType, TransformType (RTM_SIMD_CALL *mul_fn)(const TransformType&, const TransformType&)>
static void bm_vqm_mul_latency(benchmark::State& state, input_kind kind)
{
	bench_inputs<TransformType>& inputs = get_inputs<TransformType>(kind);

	for (auto _ : state)
	{
		for (size_t chain_start = 0; chain_start < k_num_transforms; chain_start += k_chain_length)
		{
			TransformType parent_to_world = inputs.rhs[chain_start];
			for (size_t index = chain_start; index < chain_start + k_chain_length; ++index)
				parent_to_world = mul_fn(inputs.lhs[index], parent_to_world);

			inputs.results[chain_start] = parent_to_world;
		}

		benchmark::ClobberMemory();
	}

	state.SetItemsProcessed(int64_t(state.iterations()) * int64_t(k_num_transforms));
}

template<typename TransformType, TransformType (RTM_SIMD_CALL *inverse_fn)(const TransformType&)>
static void bm_vqm_inverse_throughput(benchmark::State& state, input_kind kind)
{
	bench_inputs<TransformType>& inputs = get_inputs<TransformType>(kind);

	for (auto _ : state)
	{
		for (size_t index = 0; index < k_num_transforms; ++index)
			inputs.results[index] = inverse_fn(inputs.lhs[index]);

		benchmark::ClobberMemory();
	}

	state.SetItemsProcessed(int64_t(state.iterations()) * int64_t(k_num_transforms));
}

template<typename TransformType>
static void bm_vqm_mul_point3_throughput(benchmark::State& state, input_kind kind)
{
	bench_inputs<TransformType>& inputs = get_inputs<TransformType>(kind);

	for (auto _ : state)
	{
		for (size_t index = 0; index < k_num_transforms; ++index)
			inputs.point_results[index] = vqm_mul_point3_ref<TransformType>(inputs.points[index], inputs.lhs[index]);

		benchmark::ClobberMemory();
	}

	state.SetItemsProcessed(int64_t(state.iterations()) * int64_t(k_num_transforms));
}

template<typename TransformType>
static void register_vqm_benchmarks(const char* type_name)
{
	const input_kind kinds[] = { input_kind::rigid, input_kind::general, input_kind::mixed };
	for (input_kind kind : kinds)
	{
		const std::string suffix = std::string("/") + get_input_kind_name(kind);
		const std::string prefix = std::string("bm_") + type_name;

		benchmark::RegisterBenchmark((prefix + "_mul_ref/throughput" + suffix).c_str(), bm_vqm_mul_throughput<TransformType, vqm_mul_ref<TransformType>>, kind);
		benchmark::RegisterBenchmark((prefix + "_mul_quat/throughput" + suffix).c_str(), bm_vqm_mul_throughput<TransformType, vqm_mul_quat<TransformType>>, kind);
		benchmark::RegisterBenchmark((prefix + "_mul_soa_cross/throughput" + suffix).c_str(), bm_vqm_mul_throughput<TransformType, vqm_mul_soa<TransformType, false>>, kind);
		benchmark::RegisterBenchmark((prefix + "_mul_soa_quat/throughput" + suffix).c_str(), bm_vqm_mul_throughput<TransformType, vqm_mul_soa<TransformType, true>>, kind);
		benchmark::RegisterBenchmark((prefix + "_mul_soa_quat_split/throughput" + suffix).c_str(), bm_vqm_mul_throughput<TransformType, vqm_mul_soa_split<TransformType>>, kind);

		benchmark::RegisterBenchmark((prefix + "_mul_ref/latency" + suffix).c_str(), bm_vqm_mul_latency<TransformType, vqm_mul_ref<TransformType>>, kind);
		benchmark::RegisterBenchmark((prefix + "_mul_quat/latency" + suffix).c_str(), bm_vqm_mul_latency<TransformType, vqm_mul_quat<TransformType>>, kind);
		benchmark::RegisterBenchmark((prefix + "_mul_soa_cross/latency" + suffix).c_str(), bm_vqm_mul_latency<TransformType, vqm_mul_soa<TransformType, false>>, kind);
		benchmark::RegisterBenchmark((prefix + "_mul_soa_quat/latency" + suffix).c_str(), bm_vqm_mul_latency<TransformType, vqm_mul_soa<TransformType, true>>, kind);
		benchmark::RegisterBenchmark((prefix + "_mul_soa_quat_split/latency" + suffix).c_str(), bm_vqm_mul_latency<TransformType, vqm_mul_soa_split<TransformType>>, kind);

		benchmark::RegisterBenchmark((prefix + "_inverse_ref/throughput" + suffix).c_str(), bm_vqm_inverse_throughput<TransformType, vqm_inverse_ref<TransformType>>, kind);
		benchmark::RegisterBenchmark((prefix + "_inverse_quat/throughput" + suffix).c_str(), bm_vqm_inverse_throughput<TransformType, vqm_inverse_quat<TransformType>>, kind);
		benchmark::RegisterBenchmark((prefix + "_inverse_soa_cross/throughput" + suffix).c_str(), bm_vqm_inverse_throughput<TransformType, vqm_inverse_soa<TransformType, false>>, kind);
		benchmark::RegisterBenchmark((prefix + "_inverse_soa_quat/throughput" + suffix).c_str(), bm_vqm_inverse_throughput<TransformType, vqm_inverse_soa<TransformType, true>>, kind);

		benchmark::RegisterBenchmark((prefix + "_mul_point3_ref/throughput" + suffix).c_str(), bm_vqm_mul_point3_throughput<TransformType>, kind);
	}
}

static int register_all_vqm_benchmarks()
{
	register_vqm_benchmarks<vqmf>("vqmf");
	register_vqm_benchmarks<vqmd>("vqmd");
	return 0;
}

static const int k_vqm_benchmarks_registered = register_all_vqm_benchmarks();
