/**************************************************************************/
/*  proxy_cloth.h                                                         */
/**************************************************************************/

#pragma once

#include "core/object/ref_counted.h"
#include "core/templates/local_vector.h"
#include "core/variant/typed_array.h"

// Coarse XPBD cloth for a character's loose garment (vr-devkit near tier). Particles follow a skinned target pose
// (the spring-bone pose) within a per-particle max distance, with stretch and bend constraints, gravity, inertia and
// capsule collision, stepped with substeps at the rendered frame rate. The render mesh is the target-skinned mesh
// plus the particles' offsets from their targets, spread by fixed inverse-distance weights, so blend 0 reproduces
// the spring-bone mesh exactly (seamless LOD hand-off).
class ProxyCloth : public RefCounted {
	GDCLASS(ProxyCloth, RefCounted);

	struct Skin {
		int bone[4] = { 0, 0, 0, 0 };
		float weight[4] = { 0, 0, 0, 0 };
	};
	struct Link {
		int a = 0;
		int b = 0;
		float rest = 0;
		float compliance = 0;
	};

	LocalVector<Vector3> rest; // particle rest positions (mesh space)
	LocalVector<Skin> skin;
	LocalVector<Vector3> x, prev, target, last_target;
	LocalVector<float> inv_mass, max_distance;
	LocalVector<Link> links;

	LocalVector<Vector3> render_rest, render_normal_rest, render_tangent_rest;
	LocalVector<Skin> render_skin;
	LocalVector<int> render_tris;
	LocalVector<int> wrap_particle; // 4 per render vertex
	LocalVector<float> wrap_weight;

	LocalVector<Vector3> cap_a, cap_b;
	LocalVector<float> cap_r;

	PackedVector3Array last_render; // render positions written by the last write_mesh()
	float blend = 1.0;
	float thickness = 0.01;
	float damping = 0.02;
	bool fresh = true;
	uint64_t last_usec = 0;

	static Skin _skin_from(const PackedInt32Array &p_bones, const PackedFloat32Array &p_weights, int p_index);
	static Vector3 _apply(const Skin &p_skin, const LocalVector<Transform3D> &p_xf, const Vector3 &p_v);
	void _collide(Vector3 &r_p) const;

protected:
	static void _bind_methods();

public:
	void setup(const PackedVector3Array &p_rest, const PackedInt32Array &p_bones, const PackedFloat32Array &p_weights,
			const PackedFloat32Array &p_max_distance, const PackedInt32Array &p_stretch, const PackedInt32Array &p_bend,
			float p_stretch_compliance, float p_bend_compliance);
	void bind_render(const PackedVector3Array &p_rest, const PackedVector3Array &p_normals, const PackedFloat32Array &p_tangents,
			const PackedInt32Array &p_bones, const PackedFloat32Array &p_weights, const PackedInt32Array &p_indices);
	void set_capsules(const PackedVector3Array &p_ends, const PackedFloat32Array &p_radii);
	void step(const TypedArray<Transform3D> &p_skin_transforms, float p_delta, int p_substeps, const Vector3 &p_gravity);
	void reset();
	Error write_mesh(const RID &p_mesh, int p_surface, const TypedArray<Transform3D> &p_skin_transforms);

	PackedVector3Array get_positions() const;
	PackedVector3Array get_targets() const;
	PackedVector3Array get_render_positions() const { return last_render; }
	int get_particle_count() const { return x.size(); }
	int get_link_count() const { return links.size(); }
	float get_last_step_usec() const { return last_usec; }

	void set_blend(float p_blend) { blend = CLAMP(p_blend, 0.0f, 1.0f); }
	float get_blend() const { return blend; }
	void set_thickness(float p_thickness) { thickness = p_thickness; }
	float get_thickness() const { return thickness; }
	void set_damping(float p_damping) { damping = CLAMP(p_damping, 0.0f, 1.0f); }
	float get_damping() const { return damping; }
};
