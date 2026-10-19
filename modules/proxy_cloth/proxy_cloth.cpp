/**************************************************************************/
/*  proxy_cloth.cpp                                                       */
/**************************************************************************/

#include "proxy_cloth.h"

#include "core/object/class_db.h"
#include "core/os/os.h"
#include "servers/rendering/rendering_server.h"

ProxyCloth::Skin ProxyCloth::_skin_from(const PackedInt32Array &p_bones, const PackedFloat32Array &p_weights, int p_index) {
	Skin s;
	float total = 0;
	for (int j = 0; j < 4; j++) {
		s.bone[j] = p_bones[p_index * 4 + j];
		s.weight[j] = p_weights[p_index * 4 + j];
		total += s.weight[j];
	}
	if (total > 0) {
		for (int j = 0; j < 4; j++) {
			s.weight[j] /= total;
		}
	}
	return s;
}

Vector3 ProxyCloth::_apply(const Skin &p_skin, const LocalVector<Transform3D> &p_xf, const Vector3 &p_v) {
	Vector3 out;
	for (int j = 0; j < 4; j++) {
		if (p_skin.weight[j] > 0 && p_skin.bone[j] < (int)p_xf.size()) {
			out += p_xf[p_skin.bone[j]].xform(p_v) * p_skin.weight[j];
		}
	}
	return out;
}

void ProxyCloth::_collide(Vector3 &r_p) const {
	for (uint32_t c = 0; c < cap_r.size(); c++) {
		const Vector3 ab = cap_b[c] - cap_a[c];
		const float t = CLAMP((r_p - cap_a[c]).dot(ab) / MAX(ab.length_squared(), 1e-8f), 0.0f, 1.0f);
		const Vector3 q = cap_a[c] + ab * t;
		const Vector3 d = r_p - q;
		const float r = cap_r[c] + thickness;
		const float len = d.length();
		if (len < r && len > 1e-6f) {
			r_p = q + d * (r / len);
		}
	}
}

void ProxyCloth::setup(const PackedVector3Array &p_rest, const PackedInt32Array &p_bones, const PackedFloat32Array &p_weights,
		const PackedFloat32Array &p_max_distance, const PackedInt32Array &p_stretch, const PackedInt32Array &p_bend,
		float p_stretch_compliance, float p_bend_compliance) {
	const int n = p_rest.size();
	ERR_FAIL_COND(p_bones.size() != n * 4 || p_weights.size() != n * 4 || p_max_distance.size() != n);
	ERR_FAIL_COND(p_stretch.size() % 2 != 0 || p_bend.size() % 2 != 0);
	rest.resize(n);
	skin.resize(n);
	max_distance.resize(n);
	inv_mass.resize(n);
	for (int i = 0; i < n; i++) {
		rest[i] = p_rest[i];
		skin[i] = _skin_from(p_bones, p_weights, i);
		max_distance[i] = p_max_distance[i];
		inv_mass[i] = p_max_distance[i] > 0 ? 1.0f : 0.0f; // max distance 0: the particle is pinned to its target
	}
	links.clear();
	for (int pass = 0; pass < 2; pass++) {
		const PackedInt32Array &pairs = pass == 0 ? p_stretch : p_bend;
		for (int i = 0; i < pairs.size(); i += 2) {
			const int a = pairs[i];
			const int b = pairs[i + 1];
			ERR_CONTINUE(a < 0 || b < 0 || a >= n || b >= n || a == b);
			links.push_back({ a, b, rest[a].distance_to(rest[b]), pass == 0 ? p_stretch_compliance : p_bend_compliance });
		}
	}
	x.resize(n);
	prev.resize(n);
	target.resize(n);
	last_target.resize(n);
	fresh = true;
}

void ProxyCloth::bind_render(const PackedVector3Array &p_rest, const PackedVector3Array &p_normals, const PackedFloat32Array &p_tangents,
		const PackedInt32Array &p_bones, const PackedFloat32Array &p_weights, const PackedInt32Array &p_indices) {
	const int n = p_rest.size();
	ERR_FAIL_COND(p_bones.size() != n * 4 || p_weights.size() != n * 4 || p_normals.size() != n);
	ERR_FAIL_COND(!p_tangents.is_empty() && p_tangents.size() != n * 4);
	ERR_FAIL_COND_MSG(rest.is_empty(), "ProxyCloth: call setup() before bind_render().");
	render_rest.resize(n);
	render_normal_rest.resize(n);
	render_tangent_rest.resize(p_tangents.is_empty() ? 0 : n);
	render_skin.resize(n);
	wrap_particle.resize(n * 4);
	wrap_weight.resize(n * 4);
	for (int i = 0; i < n; i++) {
		render_rest[i] = p_rest[i];
		render_normal_rest[i] = p_normals[i];
		if (!p_tangents.is_empty()) {
			render_tangent_rest[i] = Vector3(p_tangents[i * 4], p_tangents[i * 4 + 1], p_tangents[i * 4 + 2]);
		}
		render_skin[i] = _skin_from(p_bones, p_weights, i);
		// The 4 nearest particles, by inverse squared distance (brute force: once, a few thousand by a few hundred).
		int best[4] = { 0, 0, 0, 0 };
		float dist[4] = { 1e30f, 1e30f, 1e30f, 1e30f };
		for (uint32_t p = 0; p < rest.size(); p++) {
			const float d = rest[p].distance_squared_to(p_rest[i]);
			for (int k = 0; k < 4; k++) {
				if (d < dist[k]) {
					for (int m = 3; m > k; m--) {
						dist[m] = dist[m - 1];
						best[m] = best[m - 1];
					}
					dist[k] = d;
					best[k] = p;
					break;
				}
			}
		}
		float total = 0;
		for (int k = 0; k < 4; k++) {
			const int cnt = MIN((int)rest.size(), 4);
			const float w = k < cnt ? 1.0f / (dist[k] + 1e-6f) : 0.0f;
			wrap_particle[i * 4 + k] = best[k];
			wrap_weight[i * 4 + k] = w;
			total += w;
		}
		for (int k = 0; k < 4; k++) {
			wrap_weight[i * 4 + k] /= total;
		}
	}
	render_tris.resize(p_indices.size());
	for (int i = 0; i < p_indices.size(); i++) {
		render_tris[i] = p_indices[i];
	}
}

void ProxyCloth::set_capsules(const PackedVector3Array &p_ends, const PackedFloat32Array &p_radii) {
	ERR_FAIL_COND(p_ends.size() != p_radii.size() * 2);
	cap_a.resize(p_radii.size());
	cap_b.resize(p_radii.size());
	cap_r.resize(p_radii.size());
	for (int c = 0; c < p_radii.size(); c++) {
		cap_a[c] = p_ends[c * 2];
		cap_b[c] = p_ends[c * 2 + 1];
		cap_r[c] = p_radii[c];
	}
}

void ProxyCloth::reset() {
	fresh = true;
}

void ProxyCloth::step(const TypedArray<Transform3D> &p_skin_transforms, float p_delta, int p_substeps, const Vector3 &p_gravity) {
	const uint64_t t0 = OS::get_singleton()->get_ticks_usec();
	const int n = x.size();
	LocalVector<Transform3D> xf;
	xf.resize(p_skin_transforms.size());
	for (int i = 0; i < p_skin_transforms.size(); i++) {
		xf[i] = p_skin_transforms[i];
	}
	for (int i = 0; i < n; i++) {
		last_target[i] = target[i];
		target[i] = _apply(skin[i], xf, rest[i]);
	}
	if (fresh || p_delta <= 0) {
		for (int i = 0; i < n; i++) {
			x[i] = target[i];
			prev[i] = target[i];
			last_target[i] = target[i];
		}
		fresh = false;
		last_usec = OS::get_singleton()->get_ticks_usec() - t0;
		return;
	}
	const int steps = MAX(p_substeps, 1);
	const float h = MIN(p_delta, 1.0f / 30.0f) / steps; // a hitch is simulated as a 30 Hz frame, not one huge step
	for (int s = 0; s < steps; s++) {
		const float k = float(s + 1) / steps;
		for (int i = 0; i < n; i++) {
			const Vector3 goal = last_target[i].lerp(target[i], k);
			if (inv_mass[i] == 0) {
				prev[i] = x[i];
				x[i] = goal;
				continue;
			}
			const Vector3 v = (x[i] - prev[i]) * (1.0f - damping);
			prev[i] = x[i];
			x[i] += v + p_gravity * h * h;
		}
		// One XPBD pass per substep (small steps beat many iterations); compliance scaled by the substep.
		for (const Link &l : links) {
			const float wsum = inv_mass[l.a] + inv_mass[l.b];
			if (wsum == 0) {
				continue;
			}
			const Vector3 d = x[l.a] - x[l.b];
			const float len = d.length();
			if (len < 1e-7f) {
				continue;
			}
			const float c = len - l.rest;
			const float dl = -c / (wsum + l.compliance / (h * h));
			const Vector3 corr = d * (dl / len);
			x[l.a] += corr * inv_mass[l.a];
			x[l.b] -= corr * inv_mass[l.b];
		}
		for (int i = 0; i < n; i++) {
			if (inv_mass[i] == 0) {
				continue;
			}
			const Vector3 goal = last_target[i].lerp(target[i], k);
			const Vector3 d = x[i] - goal;
			const float len = d.length();
			if (len > max_distance[i]) {
				x[i] = goal + d * (max_distance[i] / len);
			}
			_collide(x[i]);
		}
	}
	last_usec = OS::get_singleton()->get_ticks_usec() - t0;
}

Error ProxyCloth::write_mesh(const RID &p_mesh, int p_surface, const TypedArray<Transform3D> &p_skin_transforms) {
	const int n = render_rest.size();
	ERR_FAIL_COND_V_MSG(n == 0, ERR_UNCONFIGURED, "ProxyCloth: call bind_render() first.");
	LocalVector<Transform3D> xf;
	xf.resize(p_skin_transforms.size());
	for (int i = 0; i < p_skin_transforms.size(); i++) {
		xf[i] = p_skin_transforms[i];
	}
	PackedVector3Array verts;
	verts.resize(n);
	Vector3 *vw = verts.ptrw();
	for (int i = 0; i < n; i++) {
		Vector3 off;
		for (int k = 0; k < 4; k++) {
			const int p = wrap_particle[i * 4 + k];
			off += (x[p] - target[p]) * (wrap_weight[i * 4 + k] * blend);
		}
		vw[i] = _apply(render_skin[i], xf, render_rest[i]) + off;
	}
	// Normals from the deformed triangles (area weighted); tangents skinned, then made perpendicular to them.
	LocalVector<Vector3> acc;
	acc.resize(n);
	for (int i = 0; i < n; i++) {
		acc[i] = Vector3();
	}
	for (uint32_t t = 0; t + 2 < render_tris.size(); t += 3) {
		const int a = render_tris[t], b = render_tris[t + 1], c = render_tris[t + 2];
		const Vector3 fn = (vw[c] - vw[a]).cross(vw[b] - vw[a]); // Godot winds clockwise
		acc[a] += fn;
		acc[b] += fn;
		acc[c] += fn;
	}
	PackedVector3Array normals;
	normals.resize(n);
	PackedFloat32Array tangents;
	tangents.resize(render_tangent_rest.is_empty() ? 0 : n * 4);
	for (int i = 0; i < n; i++) {
		Vector3 nn = acc[i].length_squared() > 1e-20f ? acc[i].normalized() : render_normal_rest[i];
		normals.write[i] = nn;
		if (!render_tangent_rest.is_empty()) {
			Vector3 tt;
			for (int j = 0; j < 4; j++) {
				const Skin &s = render_skin[i];
				if (s.weight[j] > 0 && s.bone[j] < (int)xf.size()) {
					tt += xf[s.bone[j]].basis.xform(render_tangent_rest[i]) * s.weight[j];
				}
			}
			tt = (tt - nn * nn.dot(tt));
			tt = tt.length_squared() > 1e-20f ? tt.normalized() : nn.get_any_perpendicular();
			tangents.write[i * 4] = tt.x;
			tangents.write[i * 4 + 1] = tt.y;
			tangents.write[i * 4 + 2] = tt.z;
			tangents.write[i * 4 + 3] = 1.0f;
		}
	}
	Array arrays;
	arrays.resize(RSE::ARRAY_MAX);
	arrays[RSE::ARRAY_VERTEX] = verts;
	arrays[RSE::ARRAY_NORMAL] = normals;
	if (!tangents.is_empty()) {
		arrays[RSE::ARRAY_TANGENT] = tangents;
	}
	RenderingServerTypes::SurfaceData sd;
	const Error err = RS::get_singleton()->mesh_create_surface_data_from_arrays(&sd, RSE::PRIMITIVE_TRIANGLES, arrays, Array(), Dictionary(), RSE::ARRAY_FLAG_USE_DYNAMIC_UPDATE);
	ERR_FAIL_COND_V(err != OK, err);
	RS::get_singleton()->mesh_surface_update_vertex_region(p_mesh, p_surface, 0, sd.vertex_data);
	last_render = verts;
	return OK;
}

PackedVector3Array ProxyCloth::get_positions() const {
	PackedVector3Array out;
	out.resize(x.size());
	for (uint32_t i = 0; i < x.size(); i++) {
		out.write[i] = target[i].lerp(x[i], blend);
	}
	return out;
}

PackedVector3Array ProxyCloth::get_targets() const {
	PackedVector3Array out;
	out.resize(target.size());
	for (uint32_t i = 0; i < target.size(); i++) {
		out.write[i] = target[i];
	}
	return out;
}

void ProxyCloth::_bind_methods() {
	ClassDB::bind_method(D_METHOD("setup", "rest", "bones", "weights", "max_distance", "stretch", "bend", "stretch_compliance", "bend_compliance"), &ProxyCloth::setup);
	ClassDB::bind_method(D_METHOD("bind_render", "rest", "normals", "tangents", "bones", "weights", "indices"), &ProxyCloth::bind_render);
	ClassDB::bind_method(D_METHOD("set_capsules", "ends", "radii"), &ProxyCloth::set_capsules);
	ClassDB::bind_method(D_METHOD("step", "skin_transforms", "delta", "substeps", "gravity"), &ProxyCloth::step);
	ClassDB::bind_method(D_METHOD("reset"), &ProxyCloth::reset);
	ClassDB::bind_method(D_METHOD("write_mesh", "mesh", "surface", "skin_transforms"), &ProxyCloth::write_mesh);
	ClassDB::bind_method(D_METHOD("get_positions"), &ProxyCloth::get_positions);
	ClassDB::bind_method(D_METHOD("get_targets"), &ProxyCloth::get_targets);
	ClassDB::bind_method(D_METHOD("get_render_positions"), &ProxyCloth::get_render_positions);
	ClassDB::bind_method(D_METHOD("get_particle_count"), &ProxyCloth::get_particle_count);
	ClassDB::bind_method(D_METHOD("get_link_count"), &ProxyCloth::get_link_count);
	ClassDB::bind_method(D_METHOD("get_last_step_usec"), &ProxyCloth::get_last_step_usec);
	ClassDB::bind_method(D_METHOD("set_blend", "blend"), &ProxyCloth::set_blend);
	ClassDB::bind_method(D_METHOD("get_blend"), &ProxyCloth::get_blend);
	ClassDB::bind_method(D_METHOD("set_thickness", "thickness"), &ProxyCloth::set_thickness);
	ClassDB::bind_method(D_METHOD("get_thickness"), &ProxyCloth::get_thickness);
	ClassDB::bind_method(D_METHOD("set_damping", "damping"), &ProxyCloth::set_damping);
	ClassDB::bind_method(D_METHOD("get_damping"), &ProxyCloth::get_damping);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "blend", PROPERTY_HINT_RANGE, "0,1,0.01"), "set_blend", "get_blend");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "thickness", PROPERTY_HINT_RANGE, "0,0.1,0.001,suffix:m"), "set_thickness", "get_thickness");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "damping", PROPERTY_HINT_RANGE, "0,1,0.01"), "set_damping", "get_damping");
}
