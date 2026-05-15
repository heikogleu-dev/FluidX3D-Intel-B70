#include "lbm.hpp"



Units units; // for unit conversion

#if defined(D2Q9)
const uint velocity_set = 9u;
const uint dimensions = 2u;
const uint transfers = 3u;
#elif defined(D3Q15)
const uint velocity_set = 15u;
const uint dimensions = 3u;
const uint transfers = 5u;
#elif defined(D3Q19)
const uint velocity_set = 19u;
const uint dimensions = 3u;
const uint transfers = 5u;
#elif defined(D3Q27)
const uint velocity_set = 27u;
const uint dimensions = 3u;
const uint transfers = 9u;
#endif // D3Q27

uint bytes_per_cell_host() { // returns the number of Bytes per cell allocated in host memory
	uint bytes_per_cell = 17u; // rho, u, flags
#ifdef FORCE_FIELD
	bytes_per_cell += 12u; // F
#endif // FORCE_FIELD
#ifdef SURFACE
	bytes_per_cell += 4u; // phi
#endif // SURFACE
#ifdef TEMPERATURE
	bytes_per_cell += 4u; // T
#endif // TEMPERATURE
	return bytes_per_cell;
}
uint bytes_per_cell_device() { // returns the number of Bytes per cell allocated in device memory
	uint bytes_per_cell = velocity_set*sizeof(fpxx)+17u; // fi, rho, u, flags
#ifdef FORCE_FIELD
	bytes_per_cell += 12u; // F
#endif // FORCE_FIELD
#ifdef SURFACE
	bytes_per_cell += 12u; // phi, mass, flags
#endif // SURFACE
#ifdef TEMPERATURE
	bytes_per_cell += 7u*sizeof(fpxx)+4u; // gi, T
#endif // TEMPERATURE
	return bytes_per_cell;
}
uint bandwidth_bytes_per_cell_device() { // returns the bandwidth in Bytes per cell per time step from/to device memory
	uint bandwidth_bytes_per_cell = velocity_set*2u*sizeof(fpxx)+1u; // lattice.set()*2*fi, flags
#ifdef UPDATE_FIELDS
	bandwidth_bytes_per_cell += 16u; // rho, u
#ifdef TEMPERATURE
	bandwidth_bytes_per_cell += 4u; // T
#endif // TEMPERATURE
#endif // UPDATE_FIELDS
#ifdef FORCE_FIELD
	bandwidth_bytes_per_cell += 12u; // F
#endif // FORCE_FIELD
#if defined(MOVING_BOUNDARIES)||defined(SURFACE)||defined(TEMPERATURE)
	bandwidth_bytes_per_cell += (velocity_set-1u)*1u; // neighbor flags have to be loaded
#endif // MOVING_BOUNDARIES, SURFACE or TEMPERATURE
#ifdef SURFACE
	bandwidth_bytes_per_cell += (1u+(2u*velocity_set-1u)*sizeof(fpxx)+8u+(velocity_set-1u)*4u) + 1u + 1u + (4u+velocity_set+4u+4u+4u); // surface_0 (flags, fi, mass, massex), surface_1 (flags), surface_2 (flags), surface_3 (rho, flags, mass, massex, phi)
#endif // SURFACE
#ifdef TEMPERATURE
	bandwidth_bytes_per_cell += 7u*2u*sizeof(fpxx); // 2*gi
#endif // TEMPERATURE
	return bandwidth_bytes_per_cell;
}
uint3 resolution(const float3 box_aspect_ratio, const uint memory) { // input: simulation box aspect ratio and VRAM occupation in MB, output: grid resolution
#ifndef D2Q9
	float memory_required = (box_aspect_ratio.x*box_aspect_ratio.y*box_aspect_ratio.z)*(float)bytes_per_cell_device()/1048576.0f; // in MB
	float scaling = cbrt((float)memory/memory_required);
	return uint3(to_uint(scaling*box_aspect_ratio.x), to_uint(scaling*box_aspect_ratio.y), to_uint(scaling*box_aspect_ratio.z));
#else // D2Q9
	float memory_required = (box_aspect_ratio.x*box_aspect_ratio.y)*(float)bytes_per_cell_device()/1048576.0f; // in MB
	float scaling = sqrt((float)memory/memory_required);
	return uint3(to_uint(scaling*box_aspect_ratio.x), to_uint(scaling*box_aspect_ratio.y), 1u);
#endif // D2Q9
}

string default_filename(const string& path, const string& name, const string& extension, const ulong t) { // generate a default filename with timestamp
	string time = "00000000"+to_string(t);
	time = substring(time, length(time)-9u, 9u);
	return (path=="" ? get_exe_path()+"export/" : path)+create_file_extension((name=="" ? "file" : name)+"-"+time, extension);
}
string default_filename(const string& name, const string& extension, const ulong t) { // generate a default filename with timestamp at exe_path/export/
	return default_filename("", name, extension, t);
}



LBM_Domain::LBM_Domain(const Device_Info& device_info, const uint Nx, const uint Ny, const uint Nz, const uint Dx, const uint Dy, const uint Dz, const int Ox, const int Oy, const int Oz, const float nu, const float fx, const float fy, const float fz, const float sigma, const float alpha, const float beta, const uint particles_N, const float particles_rho) { // constructor with manual device selection and domain offset
	this->Nx = Nx; this->Ny = Ny; this->Nz = Nz;
	this->Dx = Dx; this->Dy = Dy; this->Dz = Dz;
	this->Ox = Ox; this->Oy = Oy; this->Oz = Oz;
	this->nu = nu;
	this->fx = fx; this->fy = fy; this->fz = fz;
	this->sigma = sigma;
	this->alpha = alpha; this->beta = beta;
	this->particles_N = particles_N;
	this->particles_rho = particles_rho;
	string opencl_c_code;
#ifdef GRAPHICS
	graphics = Graphics(this);
	opencl_c_code = device_defines()+graphics.device_defines()+get_opencl_c_code();
#else // GRAPHICS
	opencl_c_code = device_defines()+get_opencl_c_code();
#endif // GRAPHICS
	this->device = Device(device_info, opencl_c_code);
	print_info("Allocating memory. This may take a few seconds.");
	allocate(device); // lbm first
#ifdef GRAPHICS
	graphics.allocate(device); // graphics after lbm
#endif // GRAPHICS
}

void LBM_Domain::allocate(Device& device) {
	const ulong N = get_N();
	fi = Memory<fpxx>(device, N, velocity_set, false);
	rho = Memory<float>(device, N, 1u, true, true, 1.0f);
	u = Memory<float>(device, N, 3u);
	flags = Memory<uchar>(device, N);
#ifdef WALL_VISC_BOOST
	wall_adj_flag = Memory<uchar>(device, N); // initialized to 0 = not wall-adjacent; set by populate_wall_adj_flag()
#endif // WALL_VISC_BOOST
	kernel_initialize = Kernel(device, N, "initialize", fi, rho, u, flags);
	kernel_stream_collide = Kernel(device, N, "stream_collide", fi, rho, u, flags, t, fx, fy, fz);
	// WALL_VISC_BOOST add_parameters MUST happen AFTER FORCE_FIELD/SURFACE/TEMPERATURE adds — order matches kernel signature in kernel.cpp
	kernel_update_fields = Kernel(device, N, "update_fields", fi, rho, u, flags, t, fx, fy, fz);

#ifdef FORCE_FIELD
	F = Memory<float>(device, N, 3u);
	object_sum = Memory<float>(device, 1u, 4u); // x, y, z, cell count
	kernel_stream_collide.add_parameters(F);
	kernel_update_fields.add_parameters(F);
	kernel_update_force_field = Kernel(device, N, "update_force_field", fi, flags, t, F);
	kernel_reset_force_field = Kernel(device, N, "reset_force_field", F);
	kernel_object_center_of_mass = Kernel(device, N, "object_center_of_mass", flags, (uchar)0u, object_sum);
	kernel_object_force = Kernel(device, N, "object_force", F, flags, (uchar)0u, object_sum);
	kernel_object_torque = Kernel(device, N, "object_torque", F, flags, (uchar)0u, 0.0f, 0.0f, 0.0f, object_sum);
#endif // FORCE_FIELD

	kernel_apply_freeslip_y = Kernel(device, N, "apply_freeslip_y", fi, flags, t); // CC#9 post-stream sym-plane
#ifdef SPONGE_LAYER
	kernel_apply_sponge_layer = Kernel(device, N, "apply_sponge_layer", fi, flags, t, 0.0f); // Phase 5a: outlet damping
#endif // SPONGE_LAYER
#ifdef MOVING_BOUNDARIES
	kernel_update_moving_boundaries = Kernel(device, N, "update_moving_boundaries", u, flags);
#endif // MOVING_BOUNDARIES
#ifdef WALL_MODEL_VEHICLE
	kernel_apply_wall_model_vehicle = Kernel(device, N, "apply_wall_model_vehicle", u, flags); // CC#10 WW wall model
#endif // WALL_MODEL_VEHICLE
#ifdef WALL_MODEL_FLOOR
	kernel_apply_wall_model_floor = Kernel(device, N, "apply_wall_model_floor", u, flags, 0.0f); // Path II.5 WW floor wall model (u_road param set at enqueue)
#endif // WALL_MODEL_FLOOR
// BOUZIDI_VEHICLE buffers + kernel are allocated lazily in LBM::compute_bouzidi_cells_active() when called from setup.
// Kernel/buffer remain default-constructed if compute_bouzidi_cells_active is not called → bouzidi_enabled stays false → enqueue is skipped.

#ifdef SURFACE
	phi = Memory<float>(device, N);
	mass = Memory<float>(device, N, 1u, false);
	massex = Memory<float>(device, N, 1u, false);
	kernel_initialize.add_parameters(mass, massex, phi);
	kernel_stream_collide.add_parameters(mass);
	kernel_surface_0 = Kernel(device, N, "surface_0", fi, rho, u, flags, mass, massex, phi, t, fx, fy, fz);
	kernel_surface_1 = Kernel(device, N, "surface_1", flags);
	kernel_surface_2 = Kernel(device, N, "surface_2", fi, rho, u, flags, t);
	kernel_surface_3 = Kernel(device, N, "surface_3", rho, flags, mass, massex, phi);
#endif // SURFACE

#ifdef TEMPERATURE
	gi = Memory<fpxx>(device, N, 7u, false);
	T = Memory<float>(device, N, 1u, true, true, 1.0f);
	kernel_initialize.add_parameters(gi, T);
	kernel_stream_collide.add_parameters(gi, T);
	kernel_update_fields.add_parameters(gi, T);
#endif // TEMPERATURE

#ifdef WALL_VISC_BOOST
	kernel_stream_collide.add_parameters(wall_adj_flag); // MUST be after FORCE_FIELD/SURFACE/TEMPERATURE (matches kernel.cpp signature order)
#endif // WALL_VISC_BOOST

#ifdef PARTICLES
	particles = Memory<float>(device, (ulong)particles_N, 3u);
	kernel_integrate_particles = Kernel(device, (ulong)particles_N, "integrate_particles", particles, u, flags, 1.0f);
#ifdef FORCE_FIELD
	kernel_integrate_particles.add_parameters(F, fx, fy, fz);
#endif // FORCE_FIELD
#endif // PARTICLES

	if(get_D()>1u) allocate_transfer(device);
}

void LBM_Domain::enqueue_initialize() { // call kernel_initialize
	kernel_initialize.enqueue_run();
}
void LBM_Domain::enqueue_stream_collide() { // call kernel_stream_collide to perform one LBM time step
	kernel_stream_collide.set_parameters(4u, t, fx, fy, fz).enqueue_run();
}
void LBM_Domain::enqueue_apply_freeslip_y() { // CC#9: post-stream specular reflection at TYPE_Y cells
	kernel_apply_freeslip_y.set_parameters(2u, t).enqueue_run();
}
#ifdef SPONGE_LAYER
void LBM_Domain::enqueue_apply_sponge_layer(float u_inlet) { // Phase 5a: outlet damping
	kernel_apply_sponge_layer.set_parameters(2u, t).set_parameters(3u, u_inlet).enqueue_run();
}
#endif // SPONGE_LAYER
#ifdef WALL_MODEL_VEHICLE
void LBM_Domain::enqueue_apply_wall_model_vehicle() { // CC#10: Werner-Wengle wall model on vehicle cells
	kernel_apply_wall_model_vehicle.enqueue_run();
}
#endif // WALL_MODEL_VEHICLE
#ifdef WALL_MODEL_FLOOR
void LBM_Domain::enqueue_apply_wall_model_floor(float u_road) { // Path II.5: WW wall model on rolling-road floor (z=0)
	kernel_apply_wall_model_floor.set_parameters(2u, u_road).enqueue_run();
}
#endif // WALL_MODEL_FLOOR
#ifdef BOUZIDI_VEHICLE
void LBM_Domain::enqueue_apply_bouzidi_sparse() { // Sparse Bouzidi sub-grid BB
	kernel_apply_bouzidi_sparse.set_parameters(5u, t).enqueue_run();
}
#endif // BOUZIDI_VEHICLE
#ifdef WALL_SLIP_VEHICLE
void LBM_Domain::enqueue_apply_wall_slip(float slip_factor, float blend_strength) { // Multi-cell u-prescription (BLEND version)
	kernel_apply_wall_slip.set_parameters(6u, t).set_parameters(7u, slip_factor).set_parameters(8u, blend_strength).enqueue_run();
}
#endif // WALL_SLIP_VEHICLE
void LBM_Domain::enqueue_update_fields() { // update fields (rho, u, T) manually
#ifndef UPDATE_FIELDS
	if(t!=t_last_update_fields) { // only run kernel_update_fields if the time step has changed since last update
		kernel_update_fields.set_parameters(4u, t, fx, fy, fz).enqueue_run();
		t_last_update_fields = t;
	}
#endif // UPDATE_FIELDS
}
#ifdef SURFACE
void LBM_Domain::enqueue_surface_0() {
	kernel_surface_0.set_parameters(7u, t, fx, fy, fz).enqueue_run();
}
void LBM_Domain::enqueue_surface_1() {
	kernel_surface_1.enqueue_run();
}
void LBM_Domain::enqueue_surface_2() {
	kernel_surface_2.set_parameters(4u, t).enqueue_run();
}
void LBM_Domain::enqueue_surface_3() {
	kernel_surface_3.enqueue_run();
}
#endif // SURFACE
#ifdef FORCE_FIELD
void LBM_Domain::enqueue_update_force_field() { // calculate forces from fluid on TYPE_S cells
	if(t!=t_last_force_field) { // only run kernel_update_force_field if the time step has changed since last update
		kernel_update_force_field.set_parameters(2u, t).enqueue_run();
		t_last_force_field = t;
	}
}
void LBM_Domain::enqueue_object_center_of_mass(const uchar flag_marker) { // calculate center of mass of all cells flagged with flag_marker
	object_sum.x[0] = 0.0f; // reset object_sum
	object_sum.y[0] = 0.0f;
	object_sum.z[0] = 0.0f;
	object_sum.enqueue_write_to_device();
	kernel_object_center_of_mass.set_parameters(1u, flag_marker).enqueue_run();
	object_sum.enqueue_read_from_device();
}
void LBM_Domain::enqueue_object_force(const uchar flag_marker) { // add up force for all cells flagged with flag_marker
	enqueue_update_force_field(); // update force field if it is not yet up-to-date
	object_sum.x[0] = 0.0f; // reset object_sum
	object_sum.y[0] = 0.0f;
	object_sum.z[0] = 0.0f;
	object_sum.enqueue_write_to_device();
	kernel_object_force.set_parameters(2u, flag_marker).enqueue_run();
	object_sum.enqueue_read_from_device();
}
void LBM_Domain::enqueue_object_torque(const float3& rotation_center, const uchar flag_marker) { // add up torque around specified rotation_center for all cells flagged with flag_marker
	enqueue_update_force_field(); // update force field if it is not yet up-to-date
	object_sum.x[0] = 0.0f; // reset object_sum
	object_sum.y[0] = 0.0f;
	object_sum.z[0] = 0.0f;
	object_sum.enqueue_write_to_device();
	kernel_object_torque.set_parameters(2u, flag_marker, rotation_center.x, rotation_center.y, rotation_center.z).enqueue_run();
	object_sum.enqueue_read_from_device();
}
#endif // FORCE_FIELD
#ifdef MOVING_BOUNDARIES
void LBM_Domain::enqueue_update_moving_boundaries() { // mark/unmark cells next to TYPE_S cells with velocity!=0 with TYPE_MS
	kernel_update_moving_boundaries.enqueue_run();
}
#endif // MOVING_BOUNDARIES
#ifdef PARTICLES
void LBM_Domain::enqueue_integrate_particles(const uint time_step_multiplicator) { // intgegrate particles forward in time and couple particles to fluid
#ifdef FORCE_FIELD
	if(particles_rho!=1.0f) kernel_reset_force_field.enqueue_run(); // only reset force field if particles have buoyancy and apply forces on fluid
	kernel_integrate_particles.set_parameters(5u, fx, fy, fz);
#endif // FORCE_FIELD
	kernel_integrate_particles.set_parameters(3u, (float)time_step_multiplicator).enqueue_run();
}
#endif // PARTICLES

void LBM_Domain::increment_time_step(const ulong steps) {
	t += steps; // increment time step
#ifdef UPDATE_FIELDS
	t_last_update_fields = t;
#endif // UPDATE_FIELDS
}
void LBM_Domain::reset_time_step() {
	t = 0ull; // increment time step
#ifdef UPDATE_FIELDS
	t_last_update_fields = t;
#endif // UPDATE_FIELDS
}
void LBM_Domain::finish_queue() {
	device.finish_queue();
}

uint LBM_Domain::get_velocity_set() const {
	return velocity_set;
}

void LBM_Domain::voxelize_mesh_on_device(const Mesh* mesh, const uchar flag, const float3& rotation_center, const float3& linear_velocity, const float3& rotational_velocity) { // voxelize triangle mesh
	Memory<float3> p0(device, mesh->triangle_number, 1u, mesh->p0);
	Memory<float3> p1(device, mesh->triangle_number, 1u, mesh->p1);
	Memory<float3> p2(device, mesh->triangle_number, 1u, mesh->p2);
	Memory<float> bounding_box_and_velocity(device, 16u);
	const float x0=mesh->pmin.x-2.0f, y0=mesh->pmin.y-2.0f, z0=mesh->pmin.z-2.0f, x1=mesh->pmax.x+2.0f, y1=mesh->pmax.y+2.0f, z1=mesh->pmax.z+2.0f; // use bounding box of mesh to speed up voxelization; add tolerance of 2 cells for re-voxelization of moving objects
	bounding_box_and_velocity[ 0] = as_float(mesh->triangle_number);
	bounding_box_and_velocity[ 1] = x0;
	bounding_box_and_velocity[ 2] = y0;
	bounding_box_and_velocity[ 3] = z0;
	bounding_box_and_velocity[ 4] = x1;
	bounding_box_and_velocity[ 5] = y1;
	bounding_box_and_velocity[ 6] = z1;
	bounding_box_and_velocity[ 7] = rotation_center.x;
	bounding_box_and_velocity[ 8] = rotation_center.y;
	bounding_box_and_velocity[ 9] = rotation_center.z;
	bounding_box_and_velocity[10] = linear_velocity.x;
	bounding_box_and_velocity[11] = linear_velocity.y;
	bounding_box_and_velocity[12] = linear_velocity.z;
	bounding_box_and_velocity[13] = rotational_velocity.x;
	bounding_box_and_velocity[14] = rotational_velocity.y;
	bounding_box_and_velocity[15] = rotational_velocity.z;
	uint direction = 0u;
	if(length(rotational_velocity)==0.0f) { // choose direction of minimum bounding-box cross-section area
		float v[3] = { (y1-y0)*(z1-z0), (z1-z0)*(x1-x0), (x1-x0)*(y1-y0) };
		float vmin = v[0];
		for(uint i=1u; i<3u; i++) {
			if(v[i]<vmin) {
				vmin = v[i];
				direction = i;
			}
		}
	} else { // choose direction closest to rotation axis
		float v[3] = { fabsf(rotational_velocity.x), fabsf(rotational_velocity.y), fabsf(rotational_velocity.z) };
		float vmax = v[0];
		for(uint i=1u; i<3u; i++) {
			if(v[i]>vmax) {
				vmax = v[i];
				direction = i; // find direction of minimum bounding-box cross-section area
			}
		}
	}
	const ulong A[3] = { (ulong)Ny*(ulong)Nz, (ulong)Nz*(ulong)Nx, (ulong)Nx*(ulong)Ny };
	Kernel kernel_voxelize_mesh(device, A[direction], "voxelize_mesh", direction, fi, u, flags, t+1ull, flag, p0, p1, p2, bounding_box_and_velocity);
#ifdef SURFACE
	kernel_voxelize_mesh.add_parameters(mass, massex);
#endif // SURFACE
	p0.write_to_device();
	p1.write_to_device();
	p2.write_to_device();
	bounding_box_and_velocity.write_to_device();
	kernel_voxelize_mesh.run();
}
void LBM_Domain::enqueue_unvoxelize_mesh_on_device(const Mesh* mesh, const uchar flag) { // remove voxelized triangle mesh from LBM grid
	const float x0=mesh->pmin.x, y0=mesh->pmin.y, z0=mesh->pmin.z, x1=mesh->pmax.x, y1=mesh->pmax.y, z1=mesh->pmax.z; // remove all flags in bounding box of mesh
	Kernel kernel_unvoxelize_mesh(device, get_N(), "unvoxelize_mesh", flags, flag, x0, y0, z0, x1, y1, z1);
	kernel_unvoxelize_mesh.run();
}

string LBM_Domain::device_defines() const { return
	"\n	#define def_Nx "+to_string(Nx)+"u"
	"\n	#define def_Ny "+to_string(Ny)+"u"
	"\n	#define def_Nz "+to_string(Nz)+"u"
	"\n	#define def_N "+to_string(get_N())+"ul"
	"\n	#define uxx "+(get_N()<=(ulong)max_uint ? "uint" : "ulong")+"" // switchable data type for index calculation (32-bit uint / 64-bit ulong)

	"\n	#define def_GNx "+to_string((Nx-2u*(uint)(Dx>1u))*Dx)+"u" // global LBM grid resolution of all domains together
	"\n	#define def_GNy "+to_string((Ny-2u*(uint)(Dy>1u))*Dy)+"u"
	"\n	#define def_GNz "+to_string((Nz-2u*(uint)(Dz>1u))*Dz)+"u"

	"\n	#define def_Dx "+to_string(Dx)+"u"
	"\n	#define def_Dy "+to_string(Dy)+"u"
	"\n	#define def_Dz "+to_string(Dz)+"u"

	"\n	#define def_Ox "+to_string(Ox)+"" // offsets are signed integer!
	"\n	#define def_Oy "+to_string(Oy)+""
	"\n	#define def_Oz "+to_string(Oz)+""

	"\n	#define def_Ax "+to_string(Ny*Nz)+"u"
	"\n	#define def_Ay "+to_string(Nz*Nx)+"u"
	"\n	#define def_Az "+to_string(Nx*Ny)+"u"

	"\n	#define def_domain_offset_x "+to_string(0.5f*(float)((int)Nx+2*Ox+(int)Dx*(2*(int)(Dx>1u)-(int)Nx)))+"f"
	"\n	#define def_domain_offset_y "+to_string(0.5f*(float)((int)Ny+2*Oy+(int)Dy*(2*(int)(Dy>1u)-(int)Ny)))+"f"
	"\n	#define def_domain_offset_z "+to_string(0.5f*(float)((int)Nz+2*Oz+(int)Dz*(2*(int)(Dz>1u)-(int)Nz)))+"f"

	"\n	#define D"+to_string(dimensions)+"Q"+to_string(velocity_set)+"" // D2Q9/D3Q15/D3Q19/D3Q27
	"\n	#define def_velocity_set "+to_string(velocity_set)+"u" // LBM velocity set (D2Q9/D3Q15/D3Q19/D3Q27)
	"\n	#define def_dimensions "+to_string(dimensions)+"u" // number spatial dimensions (2D or 3D)
	"\n	#define def_transfers "+to_string(transfers)+"u" // number of DDFs that are transferred between multiple domains

	"\n	#define def_c 0.57735027f" // lattice speed of sound c = 1/sqrt(3)*dt
	"\n	#define def_w " +to_string(1.0f/get_tau())+"f" // relaxation rate w = dt/tau = dt/(nu/c^2+dt/2) = 1/(3*nu+1/2)
	"\n	#define def_nu " +to_string(this->nu)+"f" // CC#10: kinematic viscosity in LB units (for wall model)
#if defined(D2Q9)
	"\n	#define def_w0 (1.0f/2.25f)" // center (0)
	"\n	#define def_ws (1.0f/9.0f)" // straight (1-4)
	"\n	#define def_we (1.0f/36.0f)" // edge (5-8)
#elif defined(D3Q15)
	"\n	#define def_w0 (1.0f/4.5f)" // center (0)
	"\n	#define def_ws (1.0f/9.0f)" // straight (1-6)
	"\n	#define def_wc (1.0f/72.0f)" // corner (7-14)
#elif defined(D3Q19)
	"\n	#define def_w0 (1.0f/3.0f)" // center (0)
	"\n	#define def_ws (1.0f/18.0f)" // straight (1-6)
	"\n	#define def_we (1.0f/36.0f)" // edge (7-18)
#elif defined(D3Q27)
	"\n	#define def_w0 (1.0f/3.375f)" // center (0)
	"\n	#define def_ws (1.0f/13.5f)" // straight (1-6)
	"\n	#define def_we (1.0f/54.0f)" // edge (7-18)
	"\n	#define def_wc (1.0f/216.0f)" // corner (19-26)
#endif // D3Q27

#if defined(SRT)
	"\n	#define SRT"
#elif defined(TRT)
	"\n	#define TRT"
#endif // TRT

	"\n	#define TYPE_S 0x01" // 0b00000001 // (stationary or moving) solid boundary
	"\n	#define TYPE_E 0x02" // 0b00000010 // equilibrium boundary (inflow/outflow)
	"\n	#define TYPE_T 0x04" // 0b00000100 // temperature boundary
	"\n	#define TYPE_F 0x08" // 0b00001000 // fluid
	"\n	#define TYPE_I 0x10" // 0b00010000 // interface
	"\n	#define TYPE_G 0x20" // 0b00100000 // gas
	"\n	#define TYPE_X 0x40" // 0b01000000 // reserved type X
	"\n	#define TYPE_Y 0x80" // 0b10000000 // reserved type Y

	"\n	#define TYPE_MS 0x03" // 0b00000011 // cell next to moving solid boundary
	"\n	#define TYPE_BO 0x03" // 0b00000011 // any flag bit used for boundaries (temperature excluded)
	"\n	#define TYPE_IF 0x18" // 0b00011000 // change from interface to fluid
	"\n	#define TYPE_IG 0x30" // 0b00110000 // change from interface to gas
	"\n	#define TYPE_GI 0x38" // 0b00111000 // change from gas to interface
	"\n	#define TYPE_SU 0x38" // 0b00111000 // any flag bit used for SURFACE
	"\n	#define TYPE_XY 0xC0" // 0b11000000 // any flag bit used for X or Y markers

#if defined(FP16S)
	"\n	#define fpxx half" // switchable data type (scaled IEEE-754 16-bit floating-point format: 1-5-10, exp-30, +-1.99902344, +-1.86446416E-9, +-1.81898936E-12, 3.311 digits)
	"\n	#define fpxx_copy ushort" // switchable data type for direct copying (scaled IEEE-754 16-bit floating-point format: 1-5-10, exp-30, +-1.99902344, +-1.86446416E-9, +-1.81898936E-12, 3.311 digits)
	"\n	#define load(p,o) (vload_half(o,p)*3.0517578E-5f)" // special function for loading half
	"\n	#define store(p,o,x) vstore_half_rte((x)*32768.0f,o,p)" // special function for storing half
#elif defined(FP16C)
	"\n	#define fpxx ushort" // switchable data type (custom 16-bit floating-point format: 1-4-11, exp-15, +-1.99951168, +-6.10351562E-5, +-2.98023224E-8, 3.612 digits), 12.5% slower than IEEE-754 16-bit
	"\n	#define fpxx_copy ushort" // switchable data type for direct copying (custom 16-bit floating-point format: 1-4-11, exp-15, +-1.99951168, +-6.10351562E-5, +-2.98023224E-8, 3.612 digits), 12.5% slower than IEEE-754 16-bit
	"\n	#define load(p,o) half_to_float_custom((p)[o])" // special function for loading half
	"\n	#define store(p,o,x) (p)[o]=float_to_half_custom(x)" // special function for storing half
#else // FP32
	"\n	#define fpxx float" // switchable data type (regular 32-bit float)
	"\n	#define fpxx_copy float" // switchable data type for direct copying (regular 32-bit float)
	"\n	#define load(p,o) (p)[o]" // regular float read
	"\n	#define store(p,o,x) (p)[o]=(x)" // regular float write
#endif // FP32

#ifdef UPDATE_FIELDS
	"\n	#define UPDATE_FIELDS"
#endif // UPDATE_FIELDS

#ifdef VOLUME_FORCE
	"\n	#define VOLUME_FORCE"
#endif // VOLUME_FORCE

#ifdef MOVING_BOUNDARIES
	"\n	#define MOVING_BOUNDARIES"
#endif // MOVING_BOUNDARIES

#ifdef WALL_MODEL_VEHICLE
	"\n	#define WALL_MODEL_VEHICLE" // CC#10: Werner-Wengle wall model on vehicle cells
#endif // WALL_MODEL_VEHICLE
#ifdef WALL_MODEL_FLOOR
	"\n	#define WALL_MODEL_FLOOR" // Path II.5: Werner-Wengle wall model on rolling-road floor
#endif // WALL_MODEL_FLOOR
#ifdef BOUZIDI_VEHICLE
	"\n	#define BOUZIDI_VEHICLE" // Sparse Bouzidi sub-grid BB at vehicle surface
#endif // BOUZIDI_VEHICLE
#ifdef WALL_SLIP_VEHICLE
	"\n	#define WALL_SLIP_VEHICLE" // Multi-cell u-prescription at wall-adjacent fluid cells
#endif // WALL_SLIP_VEHICLE
#ifdef WALL_VISC_BOOST
	"\n	#define WALL_VISC_BOOST" // Smagorinsky-Extension: boost local effective viscosity at wall-adjacent fluid cells
#endif // WALL_VISC_BOOST

#ifdef SPONGE_LAYER
	"\n	#define SPONGE_LAYER" // Phase 5a: non-reflecting outlet damping
	"\n	#define SPONGE_DEPTH_CELLS "+to_string(SPONGE_DEPTH_CELLS)+""
	"\n	#define SPONGE_STRENGTH "+to_string(SPONGE_STRENGTH)+"f"
#endif // SPONGE_LAYER

#ifdef EQUILIBRIUM_BOUNDARIES
	"\n	#define EQUILIBRIUM_BOUNDARIES"
#endif // EQUILIBRIUM_BOUNDARIES

#ifdef FORCE_FIELD
	"\n	#define FORCE_FIELD"
#endif // FORCE_FIELD

#ifdef SURFACE
	"\n	#define SURFACE"
	"\n	#define def_6_sigma "+to_string(6.0f*sigma)+"f" // rho_laplace = 2*o*K, rho = 1-rho_laplace/c^2 = 1-(6*o)*K
#endif // SURFACE

#ifdef TEMPERATURE
	"\n	#define TEMPERATURE"
	"\n	#define def_w_T "+to_string(1.0f/(2.0f*alpha+0.5f))+"f" // wT = dt/tauT = 1/(2*alpha+1/2), alpha = thermal diffusion coefficient
	"\n	#define def_beta "+to_string(beta)+"f" // thermal expansion coefficient
	"\n	#define def_T_avg "+to_string(T_avg)+"f" // average temperature
#endif // TEMPERATURE

#ifdef SUBGRID
	"\n	#define SUBGRID"
#endif // SUBGRID

#ifdef PARTICLES
	"\n	#define PARTICLES"
	"\n	#define def_particles_N "+to_string(particles_N)+"ul"
	"\n	#define def_particles_rho "+to_string(particles_rho)+"f"
#endif // PARTICLES
;}

#ifdef GRAPHICS
void LBM_Domain::Graphics::allocate(Device& device) {
	bitmap = Memory<int>(device, camera.width*camera.height);
	zbuffer = Memory<int>(device, camera.width*camera.height, 1u, lbm->get_D()>1u); // if there are multiple domains, allocate zbuffer also on host side
	camera_parameters = Memory<float>(device, 15u);
	kernel_clear = Kernel(device, bitmap.length(), "graphics_clear", bitmap, zbuffer);

	kernel_graphics_flags = Kernel(device, lbm->get_N(), "graphics_flags", camera_parameters, bitmap, zbuffer, lbm->flags);
	kernel_graphics_flags_mc = Kernel(device, lbm->get_N(), "graphics_flags_mc", camera_parameters, bitmap, zbuffer, lbm->flags);
	kernel_graphics_field = Kernel(device, lbm->get_D()==1u ? camera.width*camera.height : lbm->get_N(), lbm->get_D()==1u ? "graphics_field_rt" : "graphics_field", camera_parameters, bitmap, zbuffer, 0, lbm->rho, lbm->u, lbm->flags); // raytraced field visualization only works for single-GPU
	kernel_graphics_field_slice = Kernel(device, lbm->get_N(), "graphics_field_slice", camera_parameters, bitmap, zbuffer, 0, 0, 0, 0, 0, lbm->rho, lbm->u, lbm->flags);
#ifndef D2Q9
	kernel_graphics_streamline = Kernel(device, (lbm->get_Nx()/GRAPHICS_STREAMLINE_SPARSE)*(lbm->get_Ny()/GRAPHICS_STREAMLINE_SPARSE)*(lbm->get_Nz()/GRAPHICS_STREAMLINE_SPARSE), "graphics_streamline", camera_parameters, bitmap, zbuffer, 0, 0, 0, 0, 0, lbm->rho, lbm->u, lbm->flags); // 3D
#else // D2Q9
	kernel_graphics_streamline = Kernel(device, (lbm->get_Nx()/GRAPHICS_STREAMLINE_SPARSE)*(lbm->get_Ny()/GRAPHICS_STREAMLINE_SPARSE), "graphics_streamline", camera_parameters, bitmap, zbuffer, 0, 0, 0, 0, 0, lbm->rho, lbm->u, lbm->flags); // 2D
#endif // D2Q9
	kernel_graphics_q = Kernel(device, lbm->get_N(), "graphics_q", camera_parameters, bitmap, zbuffer, 0, lbm->rho, lbm->u);

#ifdef FORCE_FIELD
	kernel_graphics_flags.add_parameters(lbm->F);
	kernel_graphics_flags_mc.add_parameters(lbm->F);
#endif // FORCE_FIELD

#ifdef SURFACE
	skybox = Memory<int>(device, skybox_image->width()*skybox_image->height(), 1u, skybox_image->data());
	kernel_graphics_rasterize_phi = Kernel(device, lbm->get_N(), "graphics_rasterize_phi", camera_parameters, bitmap, zbuffer, lbm->phi);
	kernel_graphics_raytrace_phi = Kernel(device, bitmap.length(), "graphics_raytrace_phi", camera_parameters, bitmap, skybox, lbm->phi, lbm->flags);
	kernel_graphics_q.add_parameters(lbm->flags);
#endif // SURFACE

#ifdef TEMPERATURE
	kernel_graphics_field.add_parameters(lbm->T);
	kernel_graphics_field_slice.add_parameters(lbm->T);
	kernel_graphics_streamline.add_parameters(lbm->T);
	kernel_graphics_q.add_parameters(lbm->T);
#endif // TEMPERATURE

#ifdef PARTICLES
	kernel_graphics_particles = Kernel(device, lbm->particles.length(), "graphics_particles", camera_parameters, bitmap, zbuffer, lbm->particles);
#endif // PARTICLES
}

bool LBM_Domain::Graphics::update_camera() {
	camera.update_matrix();
	bool change = false;
	for(uint i=0u; i<15u; i++) {
		const float data = camera.data(i);
		change |= (camera_parameters[i]!=data);
		camera_parameters[i] = data;
	}
	return change; // return false if camera parameters remain unchanged
}
bool LBM_Domain::Graphics::enqueue_draw_frame(const int visualization_modes, const int field_mode, const int slice_mode, const int slice_x, const int slice_y, const int slice_z, const bool visualization_change) {
	const bool camera_update = update_camera();
#if defined(INTERACTIVE_GRAPHICS)||defined(INTERACTIVE_GRAPHICS_ASCII)
	if(!visualization_change&&!camera_update&&lbm->get_t()==t_last_rendered_frame) return false; // don't render a new frame if the scene hasn't changed since last frame
#endif // INTERACTIVE_GRAPHICS||INTERACTIVE_GRAPHICS_ASCII
	t_last_rendered_frame = lbm->get_t();
	if(camera_update) camera_parameters.enqueue_write_to_device(); // camera_parameters PCIe transfer and kernel_clear execution can happen simulataneously
	kernel_clear.enqueue_run();
	const int sx=slice_x-lbm->Ox, sy=slice_y-lbm->Oy, sz=slice_z-lbm->Oz; // subtract domain offsets
#ifdef SURFACE
	if((visualization_modes&VIS_PHI_RAYTRACE)&&lbm->get_D()==1u) kernel_graphics_raytrace_phi.enqueue_run(); // disable raytracing for multi-GPU (domain decomposition rendering doesn't work for raytracing)
	if(visualization_modes&VIS_PHI_RASTERIZE) kernel_graphics_rasterize_phi.enqueue_run();
#endif // SURFACE
	if(visualization_modes&VIS_FLAG_LATTICE) kernel_graphics_flags.enqueue_run();
	if(visualization_modes&VIS_FLAG_SURFACE) kernel_graphics_flags_mc.enqueue_run();
	if(visualization_modes&VIS_STREAMLINES) kernel_graphics_streamline.set_parameters(3u, field_mode, slice_mode, sx, sy, sz).enqueue_run();
	if(visualization_modes&VIS_Q_CRITERION) kernel_graphics_q.set_parameters(3u, field_mode).enqueue_run();
#ifdef PARTICLES
	if(visualization_modes&VIS_PARTICLES) kernel_graphics_particles.enqueue_run();
#endif // PARTICLES
	if(visualization_modes&VIS_FIELD) {
		switch(slice_mode) { // 0 (no slice), 1 (x), 2 (y), 3 (z), 4 (xz), 5 (xyz), 6 (yz), 7 (xy)
			case 0: // no slice
				kernel_graphics_field.set_parameters(3u, field_mode).enqueue_run();
				break;
			case 1: case 2: case 3: // x/y/z
				kernel_graphics_field_slice.set_ranges(lbm->get_area((uint)clamp(slice_mode-1, 0, 2))).set_parameters(3u, field_mode, slice_mode, sx, sy, sz).enqueue_run();
				break;
			case 4: // xz
				kernel_graphics_field_slice.set_ranges(lbm->get_area(0u)).set_parameters(3u, field_mode, 0u+1u, sx, sy, sz).enqueue_run();
				kernel_graphics_field_slice.set_ranges(lbm->get_area(2u)).set_parameters(3u, field_mode, 2u+1u, sx, sy, sz).enqueue_run();
				break;
			case 5: // xyz
				kernel_graphics_field_slice.set_ranges(lbm->get_area(0u)).set_parameters(3u, field_mode, 0u+1u, sx, sy, sz).enqueue_run();
				kernel_graphics_field_slice.set_ranges(lbm->get_area(1u)).set_parameters(3u, field_mode, 1u+1u, sx, sy, sz).enqueue_run();
				kernel_graphics_field_slice.set_ranges(lbm->get_area(2u)).set_parameters(3u, field_mode, 2u+1u, sx, sy, sz).enqueue_run();
				break;
			case 6: // yz
				kernel_graphics_field_slice.set_ranges(lbm->get_area(1u)).set_parameters(3u, field_mode, 1u+1u, sx, sy, sz).enqueue_run();
				kernel_graphics_field_slice.set_ranges(lbm->get_area(2u)).set_parameters(3u, field_mode, 2u+1u, sx, sy, sz).enqueue_run();
				break;
			case 7: // xy
				kernel_graphics_field_slice.set_ranges(lbm->get_area(0u)).set_parameters(3u, field_mode, 0u+1u, sx, sy, sz).enqueue_run();
				kernel_graphics_field_slice.set_ranges(lbm->get_area(1u)).set_parameters(3u, field_mode, 1u+1u, sx, sy, sz).enqueue_run();
				break;
		}
	}
	bitmap.enqueue_read_from_device();
	if(lbm->get_D()>1u) zbuffer.enqueue_read_from_device();
	return true; // new frame has been rendered
}
int* LBM_Domain::Graphics::get_bitmap() { // returns pointer to zbuffer
	return bitmap.data();
}
int* LBM_Domain::Graphics::get_zbuffer() { // returns pointer to zbuffer
	return zbuffer.data();
}

string LBM_Domain::Graphics::device_defines() const { return
	"\n	#define GRAPHICS"
	"\n	#define def_background_color " +to_string(GRAPHICS_BACKGROUND_COLOR)+""
	"\n	#define def_screen_width "     +to_string(camera.width)+"u"
	"\n	#define def_screen_height "    +to_string(camera.height)+"u"
	"\n	#define def_scale_u "          +to_string(1.0f/(0.57735027f*(GRAPHICS_U_MAX)))+"f"
	"\n	#define def_scale_rho "        +to_string(0.5f/(GRAPHICS_RHO_DELTA))+"f"
	"\n	#define def_scale_T "          +to_string(0.5f/(GRAPHICS_T_DELTA))+"f"
	"\n	#define def_scale_F "          +to_string(0.5f/(GRAPHICS_F_MAX))+"f"
	"\n	#define def_scale_Q_min "      +to_string(GRAPHICS_Q_CRITERION)+"f"
	"\n	#define def_streamline_sparse "+to_string(GRAPHICS_STREAMLINE_SPARSE)+"u"
	"\n	#define def_streamline_length "+to_string(GRAPHICS_STREAMLINE_LENGTH)+"u"
	"\n	#define def_n "                +to_string(1.333f)+"f" // refractive index of water for raytracing graphics
	"\n	#define def_attenuation "      +to_string(ln(clamp(GRAPHICS_RAYTRACING_TRANSMITTANCE, 1E-9f, 1.0f))/(float)max(max(lbm->get_Nx(), lbm->get_Ny()), lbm->get_Nz()))+"f" // (negative) attenuation parameter for raytracing graphics
	"\n	#define def_absorption_color " +to_string(GRAPHICS_RAYTRACING_COLOR)+"" // absorption color of fluid for raytracing graphics

	"\n	#define COLOR_S (127<<16|127<<8|127)" // (stationary or moving) solid boundary
	"\n	#define COLOR_E (  0<<16|255<<8|  0)" // equilibrium boundary (inflow/outflow)
	"\n	#define COLOR_M (255<<16|  0<<8|255)" // cells next to moving solid boundary
	"\n	#define COLOR_T (255<<16|  0<<8|  0)" // temperature boundary
	"\n	#define COLOR_F (  0<<16|  0<<8|255)" // fluid
	"\n	#define COLOR_I (  0<<16|255<<8|255)" // interface
	"\n	#define COLOR_0 (127<<16|127<<8|127)" // regular cell or gas
	"\n	#define COLOR_X (255<<16|127<<8|  0)" // reserved type X
	"\n	#define COLOR_Y (255<<16|255<<8|  0)" // reserved type Y
	"\n	#define COLOR_P (255<<16|255<<8|191)" // particles

#ifdef GRAPHICS_TRANSPARENCY
	"\n	#define GRAPHICS_TRANSPARENCY "+to_string(GRAPHICS_TRANSPARENCY)+"f"
#endif // GRAPHICS_TRANSPARENCY

#ifndef SURFACE
	"\n	#define def_skybox_width 1u"
	"\n	#define def_skybox_height 1u"
#else // SURFACE
	"\n	#define def_skybox_width " +to_string(skybox_image->width() )+"u"
	"\n	#define def_skybox_height "+to_string(skybox_image->height())+"u"
#endif // SURFACE
;}
#endif // GRAPHICS



vector<Device_Info> smart_device_selection(const uint D) {
	const vector<Device_Info>& devices = get_devices(); // a vector of all available OpenCL devices
	vector<Device_Info> device_infos(D);
	const int user_specified_devices = (int)main_arguments.size();
	if(user_specified_devices>0) { // user has selevted specific devices as command line arguments
		if(user_specified_devices==D) { // as much specified devices as domains
			for(uint d=0; d<D; d++) device_infos[d] = select_device_with_id(to_uint(main_arguments[d]), devices); // use list of devices IDs specified by user
		} else {
			print_warning("Incorrect number of devices specified. Using single fastest device for all domains.");
			for(uint d=0; d<D; d++) device_infos[d] = select_device_with_most_flops(devices);
		}
	} else { // device auto-selection
		vector<vector<Device_Info>> device_type_ids; // a vector of all different devices, containing vectors of their device IDs
		for(uint i=0u; i<(uint)devices.size(); i++) {
			const string name_i = devices[i].name;
			bool already_exists = false;
			for(uint j=0u; j<(uint)device_type_ids.size(); j++) {
				const string name_j = device_type_ids[j][0].name;
				if(name_i==name_j) {
					device_type_ids[j].push_back(devices[i]);
					already_exists = true;
				}
			}
			if(!already_exists) device_type_ids.push_back(vector<Device_Info>(1, devices[i]));
		}
		float best_value = -1.0f;
		int best_j = -1;
		for(uint j=0u; j<(uint)device_type_ids.size(); j++) {
			const float value = device_type_ids[j][0].tflops;
			if((uint)device_type_ids[j].size()>=D && value>best_value) {
				best_value = value;
				best_j = j;
			}
		}
		if(best_j>=0) { // select all devices of fastest device type with at least D devices of the same type
			for(uint d=0; d<D; d++) device_infos[d] = device_type_ids[best_j][d];
		} else {
			print_warning("Not enough devices of the same type available. Using single fastest device for all domains.");
			for(uint d=0; d<D; d++) device_infos[d] = select_device_with_most_flops(devices);
		}
		//for(uint j=0u; j<(uint)device_type_ids.size(); j++) print_info("Device Type "+to_string(j)+" ("+device_type_ids[j][0].name+"): "+to_string((uint)device_type_ids[j].size())+"x");
	}
	return device_infos;
}

LBM::LBM(const uint Nx, const uint Ny, const uint Nz, const float nu, const float fx, const float fy, const float fz, const float sigma, const float alpha, const float beta, const uint particles_N, const float particles_rho) // single device
	:LBM(Nx, Ny, Nz, 1u, 1u, 1u, nu, fx, fy, fz, sigma, alpha, beta, particles_N, particles_rho) { // delegating constructor
}
LBM::LBM(const uint Nx, const uint Ny, const uint Nz, const float nu, const float fx, const float fy, const float fz, const uint particles_N, const float particles_rho)
	:LBM(Nx, Ny, Nz, 1u, 1u, 1u, nu, fx, fy, fz, 0.0f, 0.0f, 0.0f, particles_N, particles_rho) { // delegating constructor
}
LBM::LBM(const uint Nx, const uint Ny, const uint Nz, const float nu, const uint particles_N, const float particles_rho)
	:LBM(Nx, Ny, Nz, 1u, 1u, 1u, nu, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, particles_N, particles_rho) { // delegating constructor
}
LBM::LBM(const uint3 N, const uint Dx, const uint Dy, const uint Dz, const float nu, const float fx, const float fy, const float fz, const float sigma, const float alpha, const float beta, const uint particles_N, const float particles_rho)
	:LBM(N.x, N.y, N.z, Dx, Dy, Dz, nu, fx, fy, fz, sigma, alpha, beta, particles_N, particles_rho) { // delegating constructor
}
LBM::LBM(const uint3 N, const float nu, const float fx, const float fy, const float fz, const float sigma, const float alpha, const float beta, const uint particles_N, const float particles_rho) // single device
	:LBM(N.x, N.y, N.z, 1u, 1u, 1u, nu, fx, fy, fz, sigma, alpha, beta, particles_N, particles_rho) { // delegating constructor
}
LBM::LBM(const uint3 N, const float nu, const uint particles_N, const float particles_rho)
	:LBM(N.x, N.y, N.z, 1u, 1u, 1u, nu, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, particles_N, particles_rho) { // delegating constructor
}
LBM::LBM(const uint3 N, const float nu, const float fx, const float fy, const float fz, const uint particles_N, const float particles_rho)
	:LBM(N.x, N.y, N.z, 1u, 1u, 1u, nu, fx, fy, fz, 0.0f, 0.0f, 0.0f, particles_N, particles_rho) { // delegating constructor
}
LBM::LBM(const uint Nx, const uint Ny, const uint Nz, const uint Dx, const uint Dy, const uint Dz, const float nu, const float fx, const float fy, const float fz, const float sigma, const float alpha, const float beta, const uint particles_N, const float particles_rho) { // multiple devices
	const uint NDx=(Nx/Dx)*Dx, NDy=(Ny/Dy)*Dy, NDz=(Nz/Dz)*Dz; // make resolution equally divisible by domains
	if(NDx!=Nx||NDy!=Ny||NDz!=Nz) print_warning("LBM grid ("+to_string(Nx)+"x"+to_string(Ny)+"x"+to_string(Nz)+") is not equally divisible in domains ("+to_string(Dx)+"x"+to_string(Dy)+"x"+to_string(Dz)+"). Changing resolution to ("+to_string(NDx)+"x"+to_string(NDy)+"x"+to_string(NDz)+").");
	this->Nx = NDx; this->Ny = NDy; this->Nz = NDz;
	this->Dx = Dx; this->Dy = Dy; this->Dz = Dz;
	const uint D = Dx*Dy*Dz;
	const uint Hx=Dx>1u, Hy=Dy>1u, Hz=Dz>1u; // halo offsets
	const vector<Device_Info>& device_infos = smart_device_selection(D);
	sanity_checks_constructor(device_infos, this->Nx, this->Ny, this->Nz, Dx, Dy, Dz, nu, fx, fy, fz, sigma, alpha, beta, particles_N, particles_rho);
	lbm_domain = new LBM_Domain*[D];
	for(uint d=0u; d<D; d++) { // parallel_for((ulong)D, D, [&](ulong d) {
		const uint x=((uint)d%(Dx*Dy))%Dx, y=((uint)d%(Dx*Dy))/Dx, z=(uint)d/(Dx*Dy); // d = x+(y+z*Dy)*Dx
		lbm_domain[d] = new LBM_Domain(device_infos[d], this->Nx/Dx+2u*Hx, this->Ny/Dy+2u*Hy, this->Nz/Dz+2u*Hz, Dx, Dy, Dz, (int)(x*this->Nx/Dx)-(int)Hx, (int)(y*this->Ny/Dy)-(int)Hy, (int)(z*this->Nz/Dz)-(int)Hz, nu, fx, fy, fz, sigma, alpha, beta, particles_N, particles_rho);
	} // });
	{
		Memory<float>** buffers_rho = new Memory<float>*[D];
		for(uint d=0u; d<D; d++) buffers_rho[d] = &(lbm_domain[d]->rho);
		rho = Memory_Container(this, buffers_rho, "rho");
	} {
		Memory<float>** buffers_u = new Memory<float>*[D];
		for(uint d=0u; d<D; d++) buffers_u[d] = &(lbm_domain[d]->u);
		u = Memory_Container(this, buffers_u, "u");
	} {
		Memory<uchar>** buffers_flags = new Memory<uchar>*[D];
		for(uint d=0u; d<D; d++) buffers_flags[d] = &(lbm_domain[d]->flags);
		flags = Memory_Container(this, buffers_flags, "flags");
	} {
#ifdef FORCE_FIELD
		Memory<float>** buffers_F = new Memory<float>*[D];
		for(uint d=0u; d<D; d++) buffers_F[d] = &(lbm_domain[d]->F);
		F = Memory_Container(this, buffers_F, "F");
#endif // FORCE_FIELD
	} {
#ifdef SURFACE
		Memory<float>** buffers_phi = new Memory<float>*[D];
		for(uint d=0u; d<D; d++) buffers_phi[d] = &(lbm_domain[d]->phi);
		phi = Memory_Container(this, buffers_phi, "phi");
#endif // SURFACE
	} {
#ifdef TEMPERATURE
		Memory<float>** buffers_T = new Memory<float>*[D];
		for(uint d=0u; d<D; d++) buffers_T[d] = &(lbm_domain[d]->T);
		T = Memory_Container(this, buffers_T, "T");
#endif // TEMPERATURE
	} {
#ifdef PARTICLES
		particles = &(lbm_domain[0]->particles);
#endif // PARTICLES
	}
#ifdef GRAPHICS
	graphics = Graphics(this);
#endif // GRAPHICS
}
LBM::~LBM() {
#ifdef GRAPHICS
	camera.allow_rendering = false;
#endif // GRAPHICS
	info.print_finalize();
	for(uint d=0u; d<get_D(); d++) delete lbm_domain[d];
	delete[] lbm_domain;
}

void LBM::sanity_checks_constructor(const vector<Device_Info>& device_infos, const uint Nx, const uint Ny, const uint Nz, const uint Dx, const uint Dy, const uint Dz, const float nu, const float fx, const float fy, const float fz, const float sigma, const float alpha, const float beta, const uint particles_N, const float particles_rho) { // sanity checks on grid resolution and extension support
	if((ulong)Nx*(ulong)Ny*(ulong)Nz==0ull) print_error("Grid point number is 0: "+to_string(Nx)+"x"+to_string(Ny)+"x"+to_string(Nz)+" = 0.");
	if(Dx*Dy*Dz==0u) print_error("You specified 0 LBM grid domains ("+to_string(Dx)+"x"+to_string(Dy)+"x"+to_string(Dz)+"). There has to be at least 1 domain in every direction. Check your input in LBM constructor.");
	const uint local_Nx=Nx/Dx+2u*(Dx>1u), local_Ny=Ny/Dy+2u*(Dy>1u), local_Nz=Nz/Dz+2u*(Dz>1u);
	uint memory_available = max_uint; // in MB
	for(Device_Info device_info : device_infos) memory_available = min(memory_available, device_info.memory);
	uint memory_required = (uint)((ulong)Nx*(ulong)Ny*(ulong)Nz/((ulong)(Dx*Dy*Dz))*(ulong)bytes_per_cell_device()/1048576ull); // in MB
	if(memory_required>memory_available) {
		float factor = cbrt((float)memory_available/(float)memory_required);
		const uint maxNx=(uint)(factor*(float)Nx), maxNy=(uint)(factor*(float)Ny), maxNz=(uint)(factor*(float)Nz);
		string message = "Grid resolution ("+to_string(Nx)+", "+to_string(Ny)+", "+to_string(Nz)+") is too large: "+to_string(Dx*Dy*Dz)+"x "+to_string(memory_required)+" MB required, "+to_string(Dx*Dy*Dz)+"x "+to_string(memory_available)+" MB available. Largest possible resolution is ("+to_string(maxNx)+", "+to_string(maxNy)+", "+to_string(maxNz)+"). Restart the simulation with lower resolution or on different device(s) with more memory.";
#if !defined(FP16S)&&!defined(FP16C)
		uint memory_required_fp16 = (uint)((ulong)Nx*(ulong)Ny*(ulong)Nz/((ulong)(Dx*Dy*Dz))*(ulong)(bytes_per_cell_device()-velocity_set*2u)/1048576ull); // in MB
		float factor_fp16 = cbrt((float)memory_available/(float)memory_required_fp16);
		const uint maxNx_fp16=(uint)(factor_fp16*(float)Nx), maxNy_fp16=(uint)(factor_fp16*(float)Ny), maxNz_fp16=(uint)(factor_fp16*(float)Nz);
		message += " Consider using FP16S/FP16C memory compression to double maximum grid resolution to a maximum of ("+to_string(maxNx_fp16)+", "+to_string(maxNy_fp16)+", "+to_string(maxNz_fp16)+"); for this, uncomment \"#define FP16S\" or \"#define FP16C\" in defines.hpp.";
#endif // !FP16S&&!FP16C
		print_error(message);
	}
	if(nu==0.0f) print_error("Viscosity cannot be 0. Change it in setup.cpp."); // sanity checks for viscosity
	else if(nu<0.0f) print_error("Viscosity cannot be negative. Remove the \"-\" in setup.cpp.");
#ifdef D2Q9
	if(Nz!=1u) print_error("D2Q9 is the 2D velocity set. You have to set Nz=1u in the LBM constructor! Currently you have set Nz="+to_string(Nz)+"u.");
#endif // D2Q9
#if !defined(SRT)&&!defined(TRT)
	print_error("No LBM collision operator selected. Uncomment either \"#define SRT\" or \"#define TRT\" in defines.hpp");
#elif defined(SRT)&&defined(TRT)
	print_error("Too many LBM collision operators selected. Comment out either \"#define SRT\" or \"#define TRT\" in defines.hpp");
#endif // SRT && TRT
#ifndef VOLUME_FORCE
	if(fx!=0.0f||fy!=0.0f||fz!=0.0f) print_error("Volume force is set in LBM constructor in main_setup(), but VOLUME_FORCE is not enabled. Uncomment \"#define VOLUME_FORCE\" in defines.hpp.");
#else // VOLUME_FORCE
#ifndef FORCE_FIELD
	if(fx==0.0f&&fy==0.0f&&fz==0.0f) print_warning("The VOLUME_FORCE extension is enabled but the volume force in LBM constructor is set to zero. You may disable the extension by commenting out \"#define VOLUME_FORCE\" in defines.hpp.");
#endif // FORCE_FIELD
#endif // VOLUME_FORCE
#ifndef SURFACE
	if(sigma!=0.0f) print_error("Surface tension is set in LBM constructor in main_setup(), but SURFACE is not enabled. Uncomment \"#define SURFACE\" in defines.hpp.");
#endif // SURFACE
#ifndef TEMPERATURE
	if(alpha!=0.0f||beta!=0.0f) print_error("Thermal diffusion/expansion coefficients are set in LBM constructor in main_setup(), but TEMPERATURE is not enabled. Uncomment \"#define TEMPERATURE\" in defines.hpp.");
#else // TEMPERATURE
	if(alpha==0.0f&&beta==0.0f) print_warning("The TEMPERATURE extension is enabled but the thermal diffusion/expansion coefficients alpha/beta in the LBM constructor are both set to zero. You may disable the extension by commenting out \"#define TEMPERATURE\" in defines.hpp.");
#endif // TEMPERATURE
#ifdef PARTICLES
	if(particles_N==0u) print_error("The PARTICLES extension is enabled but the number of particles is set to 0. Comment out \"#define PARTICLES\" in defines.hpp.");
#if !defined(VOLUME_FORCE)||!defined(FORCE_FIELD)
	if(particles_rho!=1.0f) print_error("Particle density is set unequal to 1, but particle-fluid 2-way-coupling is not enabled. Uncomment both \"#define VOLUME_FORCE\" and \"#define FORCE_FIELD\" in defines.hpp.");
#endif // !VOLUME_FORCE||!FORCE_FIELD
#ifdef FORCE_FIELD
	if(particles_rho==1.0f) print_warning("Particle density is set to 1, so particles behave as passive tracers without acting a force on the fluid, but particle-fluid 2-way-coupling is enabled. You may comment out \"#define FORCE_FIELD\" in defines.hpp.");
#endif // FORCE_FIELD
#else // PARTICLES
	if(particles_N>0u) print_error("The PARTICLES extension is disabled but the number of particles is set to "+to_string(particles_N)+">0. Uncomment \"#define PARTICLES\" in defines.hpp.");
#endif // PARTICLES
}

void LBM::sanity_checks_initialization() { // sanity checks during initialization on used extensions based on used flags
	uchar flags_used = 0u;
	bool moving_boundaries_used=false, equilibrium_boundaries_used=false, surface_used=false, temperature_used=false; // identify used extensions based used flags
	const uint threads = thread::hardware_concurrency();
	vector<uchar> t_flags_used(threads, 0u);
	vector<char> t_moving_boundaries_used(threads, false); // don't use vector<bool> as it uses bit-packing which is broken for multithreading
	vector<char> t_equilibrium_boundaries_used(threads, false); // don't use vector<bool> as it uses bit-packing which is broken for multithreading
	parallel_for(get_N(), threads, [&](ulong n, uint t) {
		const uchar flagsn = flags[n];
		const uchar flagsn_bo = flagsn&(TYPE_S|TYPE_E);
		t_flags_used[t] = t_flags_used[t]|flagsn;
		if(flagsn_bo&TYPE_S) t_moving_boundaries_used[t] = t_moving_boundaries_used[t] || (((flagsn_bo==TYPE_S)&&(u.x[n]!=0.0f||u.y[n]!=0.0f||u.z[n]!=0.0f))||(flagsn_bo==(TYPE_S|TYPE_E)));
		t_equilibrium_boundaries_used[t] = t_equilibrium_boundaries_used[t] || flagsn_bo==TYPE_E;
	});
	for(uint t=0u; t<threads; t++) {
		flags_used = flags_used|t_flags_used[t];
		moving_boundaries_used = moving_boundaries_used || t_moving_boundaries_used[t];
		equilibrium_boundaries_used = equilibrium_boundaries_used || t_equilibrium_boundaries_used[t];
	}
	surface_used = (bool)(flags_used&(TYPE_F|TYPE_I|TYPE_G));
	temperature_used = (bool)(flags_used&TYPE_T);
#ifndef MOVING_BOUNDARIES
	if(moving_boundaries_used) print_warning("Some boundary cells have non-zero velocity, but MOVING_BOUNDARIES is not enabled. If you intend to use moving boundaries, uncomment \"#define MOVING_BOUNDARIES\" in defines.hpp.");
#else // MOVING_BOUNDARIES
	if(!moving_boundaries_used) print_warning("The MOVING_BOUNDARIES extension is enabled but no moving boundary cells (TYPE_S flag and velocity unequal to zero) are placed in the simulation box. You may disable the extension by commenting out \"#define MOVING_BOUNDARIES\" in defines.hpp.");
#endif // MOVING_BOUNDARIES
#ifndef EQUILIBRIUM_BOUNDARIES
	if(equilibrium_boundaries_used) print_error("Some cells are set as equilibrium boundaries with the TYPE_E flag, but EQUILIBRIUM_BOUNDARIES is not enabled. Uncomment \"#define EQUILIBRIUM_BOUNDARIES\" in defines.hpp.");
#else // EQUILIBRIUM_BOUNDARIES
	if(!equilibrium_boundaries_used) print_warning("The EQUILIBRIUM_BOUNDARIES extension is enabled but no equilibrium boundary cells (TYPE_E flag) are placed in the simulation box. You may disable the extension by commenting out \"#define EQUILIBRIUM_BOUNDARIES\" in defines.hpp.");
#endif // EQUILIBRIUM_BOUNDARIES
#ifndef SURFACE
	if(surface_used) print_error("Some cells are set as fluid/interface/gas with the TYPE_F/TYPE_I/TYPE_G flags, but SURFACE is not enabled. Uncomment \"#define SURFACE\" in defines.hpp.");
#else // SURFACE
	if(!surface_used) print_error("The SURFACE extension is enabled but no fluid/interface/gas cells (TYPE_F/TYPE_I/TYPE_G flags) are placed in the simulation box. Disable the extension by commenting out \"#define SURFACE\" in defines.hpp.");
#endif // SURFACE
#ifndef TEMPERATURE
	if(temperature_used) print_error("Some cells are set as temperature boundary with the TYPE_T flag, but TEMPERATURE is not enabled. Uncomment \"#define TEMPERATURE\" in defines.hpp.");
#endif // TEMPERATURE
}

void LBM::initialize() { // write all data fields to device and call kernel_initialize
#ifndef BENCHMARK
	sanity_checks_initialization();
#endif // BENCHMARK

	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->rho.enqueue_write_to_device();
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->u.enqueue_write_to_device();
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->flags.enqueue_write_to_device();
#ifdef FORCE_FIELD
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->F.enqueue_write_to_device();
	communicate_F();
#endif // FORCE_FIELD
#ifdef SURFACE
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->phi.enqueue_write_to_device();
#endif // SURFACE
#ifdef TEMPERATURE
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->T.enqueue_write_to_device();
#endif // TEMPERATURE
#ifdef PARTICLES
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->particles.enqueue_write_to_device();
	communicate_particles();
#endif // PARTICLES

	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->increment_time_step(); // the communicate calls at initialization need an odd time step
	communicate_rho_u_flags();
#ifdef SURFACE
	communicate_phi_massex_flags();
#endif // SURFACE
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->enqueue_initialize(); // odd time step is baked-in the kernel
	communicate_rho_u_flags();
#ifdef SURFACE
	communicate_phi_massex_flags();
#endif // SURFACE
	communicate_fi(); // time step must be odd here
#ifdef TEMPERATURE
	communicate_T(); // T halo data is required for field_slice rendering
	communicate_gi(); // time step must be odd here
#endif // TEMPERATURE
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->finish_queue();
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->reset_time_step(); // set time step to 0 again
	initialized = true;
}

void LBM::do_time_step(const bool sync_single_gpu) { // call kernel_stream_collide to perform one LBM time step
#ifdef SURFACE
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->enqueue_surface_0();
#endif // SURFACE
#ifdef WALL_MODEL_VEHICLE
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->enqueue_apply_wall_model_vehicle(); // CC#10: Werner-Wengle wall model on vehicle cells BEFORE stream_collide
#ifdef MOVING_BOUNDARIES
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->enqueue_update_moving_boundaries(); // refresh TYPE_MS flags after wall model updates u_solid
#endif // MOVING_BOUNDARIES
#endif // WALL_MODEL_VEHICLE
#ifdef WALL_MODEL_FLOOR
	if(wall_floor_u_road != 0.0f) { // Path II.5: floor wall model (only when explicitly enabled by setup via lbm.wall_floor_u_road = u_road)
		for(uint d=0u; d<get_D(); d++) lbm_domain[d]->enqueue_apply_wall_model_floor(wall_floor_u_road);
#ifdef MOVING_BOUNDARIES
		for(uint d=0u; d<get_D(); d++) lbm_domain[d]->enqueue_update_moving_boundaries(); // refresh TYPE_MS flags after floor WW updates u
#endif // MOVING_BOUNDARIES
	}
#endif // WALL_MODEL_FLOOR
#ifdef BOUZIDI_VEHICLE
	if(bouzidi_enabled) { // Sparse Bouzidi sub-grid BB at vehicle wall-adjacent fluid cells (post-stream, applied to DDFs)
		for(uint d=0u; d<get_D(); d++) lbm_domain[d]->enqueue_apply_bouzidi_sparse();
	}
#endif // BOUZIDI_VEHICLE
#ifdef WALL_SLIP_VEHICLE
	if(wall_slip_blend > 0.0f) { // Multi-cell u-prescription: requires compute_bouzidi_cells_active() + wall_slip_blend > 0
		for(uint d=0u; d<get_D(); d++) lbm_domain[d]->enqueue_apply_wall_slip(wall_slip_factor, wall_slip_blend);
	}
#endif // WALL_SLIP_VEHICLE
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->enqueue_stream_collide(); // run LBM stream_collide kernel after domain communication
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->enqueue_apply_freeslip_y(); // CC#9: specular reflection at TYPE_Y cells, post-stream
#ifdef SPONGE_LAYER
	if(sponge_u_inlet != 0.0f) { // Phase 5a: outlet damping (only when explicitly enabled by setup via lbm.sponge_u_inlet = u_inlet)
		for(uint d=0u; d<get_D(); d++) lbm_domain[d]->enqueue_apply_sponge_layer(sponge_u_inlet);
	}
#endif // SPONGE_LAYER
#if defined(SURFACE) || defined(GRAPHICS)
	communicate_rho_u_flags(); // rho/u/flags halo data is required for SURFACE extension, and u halo data is required for Q-criterion rendering
#endif // SURFACE || GRAPHICS
#ifdef SURFACE
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->enqueue_surface_1();
	communicate_flags();
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->enqueue_surface_2();
	communicate_flags();
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->enqueue_surface_3();
	communicate_phi_massex_flags();
#endif // SURFACE
	communicate_fi();
#ifdef TEMPERATURE
#ifdef GRAPHICS
	communicate_T(); // T halo data is required for field_slice rendering
#endif // GRAPHICS
	communicate_gi();
#endif // TEMPERATURE
#ifdef PARTICLES
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->enqueue_integrate_particles(); // intgegrate particles forward in time and couple particles to fluid
	communicate_particles(); // communicate_F() is not required in do_time_step()
#endif // PARTICLES
	if(sync_single_gpu && get_D()==1u) for(uint d=0u; d<get_D(); d++) lbm_domain[d]->finish_queue(); // PERF-G: skip when called from run_async() — caller does barrier
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->increment_time_step();
}

void LBM::run_async(const ulong steps) { // PERF-G: submit `steps` LBM steps to GPU queue without per-step barrier; caller MUST call lbm.finish() before reading any device buffer
	if(!initialized) { print_error("LBM::run_async called before initialization. Call run() once first to initialize, then use run_async for subsequent chunks."); return; }
	info.append(steps, max_ulong, get_t());
	for(ulong i=1ull; i<=steps; i++) do_time_step(false);
}

void LBM::finish() { // PERF-G: wait for all queued kernels on this LBM's compute queues
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->finish_queue();
}

#ifdef WALL_VISC_BOOST
// Populate wall_adj_flag: for each cell, mark 1 if it is fluid AND has at least one TYPE_S|TYPE_X (vehicle) neighbor in D3Q19.
// Must be called AFTER voxelize_mesh_on_device. stream_collide reads this flag and boosts local viscosity.
void LBM::populate_wall_adj_flag() {
	flags.read_from_device(); // sync flags to host
	const uint Nx = get_Nx(), Ny = get_Ny(), Nz = get_Nz();
	const int cx[19] = {0,  1,-1,  0, 0,  0, 0,  1,-1,  1,-1,  0, 0,  1,-1,  1,-1,  0, 0};
	const int cy[19] = {0,  0, 0,  1,-1,  0, 0,  1,-1,  0, 0,  1,-1, -1, 1,  0, 0,  1,-1};
	const int cz[19] = {0,  0, 0,  0, 0,  1,-1,  0, 0,  1,-1,  1,-1,  0, 0, -1, 1, -1, 1};
	for(uint d=0u; d<get_D(); d++) {
		LBM_Domain* dom = lbm_domain[d];
		ulong n_walladj = 0ull;
		for(uint z=0u; z<Nz; z++) for(uint y=0u; y<Ny; y++) for(uint x=0u; x<Nx; x++) {
			const ulong n = (ulong)x + (ulong)y*(ulong)Nx + (ulong)z*(ulong)Nx*(ulong)Ny;
			dom->wall_adj_flag[n] = 0u; // default
			const uchar fn = dom->flags[n];
			if((fn & TYPE_S) != 0u) continue; // skip solid (cells with TYPE_S bit, includes vehicle TYPE_S|TYPE_X)
			// Check 18 D3Q19 neighbors for TYPE_S|TYPE_X (vehicle wall)
			for(uint i=1u; i<19u; i++) {
				const int nx = (int)x + cx[i], ny = (int)y + cy[i], nz = (int)z + cz[i];
				if(nx<0||nx>=(int)Nx||ny<0||ny>=(int)Ny||nz<0||nz>=(int)Nz) continue;
				const ulong nj = (ulong)nx + (ulong)ny*(ulong)Nx + (ulong)nz*(ulong)Nx*(ulong)Ny;
				if((dom->flags[nj] & (TYPE_S|TYPE_X)) == (TYPE_S|TYPE_X)) {
					dom->wall_adj_flag[n] = 1u;
					n_walladj++;
					break;
				}
			}
		}
		dom->wall_adj_flag.write_to_device();
		const ulong N_total = (ulong)Nx*(ulong)Ny*(ulong)Nz;
		print_info("WALL_VISC_BOOST: domain "+to_string(d)+" — "+to_string(n_walladj)+" wall-adjacent fluid cells ("+to_string(100.0f*(float)n_walladj/(float)N_total,3u)+"% of total)");
	}
}
#endif // WALL_VISC_BOOST

#ifdef BOUZIDI_VEHICLE
// Host-side: enumerate wall-adjacent fluid cells (those with at least one TYPE_S|TYPE_X neighbor), allocate sparse buffers,
// initialize q_data to q_default (Phase 1: uniform 0.5 = Pure-BB equivalent for sanity check).
// Must be called AFTER voxelize_mesh_on_device (flags array populated) and BEFORE run().
void LBM::compute_bouzidi_cells_active(const float q_default) {
	flags.read_from_device(); // ensure host flags are current
	const uint Nx = get_Nx(), Ny = get_Ny(), Nz = get_Nz();
	const ulong N = get_N();
	// D3Q19 c-vectors (matching FluidX3D convention)
	const int cx[19] = {0,  1,-1,  0, 0,  0, 0,  1,-1,  1,-1,  0, 0,  1,-1,  1,-1,  0, 0};
	const int cy[19] = {0,  0, 0,  1,-1,  0, 0,  1,-1,  0, 0,  1,-1, -1, 1,  0, 0,  1,-1};
	const int cz[19] = {0,  0, 0,  0, 0,  1,-1,  0, 0,  1,-1,  1,-1,  0, 0, -1, 1, -1, 1};
	std::vector<ulong> active_cells_host;
	std::vector<std::array<float,19>> active_q_host; // q for each direction, per active cell
	active_cells_host.reserve(N / 1000ull); // estimate 0.1% active
	for(uint z=1u; z<Nz-1u; z++) for(uint y=1u; y<Ny-1u; y++) for(uint x=1u; x<Nx-1u; x++) {
		const ulong n = (ulong)x + (ulong)y*(ulong)Nx + (ulong)z*(ulong)Nx*(ulong)Ny;
		const uchar fn = flags[n];
		if((fn & TYPE_S) != 0u) continue; // skip solid (we operate on fluid cells)
		// Check if any of 18 D3Q19 neighbors is TYPE_S|TYPE_X (vehicle)
		bool wall_adjacent = false;
		std::array<float,19> q_per_dir;
		q_per_dir.fill(q_default); // default: q=0.5 → Pure-BB equivalent in Bouzidi formula
		for(uint i=1u; i<19u; i++) {
			const int nx = (int)x + cx[i], ny = (int)y + cy[i], nz = (int)z + cz[i];
			if(nx<0||nx>=(int)Nx||ny<0||ny>=(int)Ny||nz<0||nz>=(int)Nz) continue;
			const ulong nj = (ulong)nx + (ulong)ny*(ulong)Nx + (ulong)nz*(ulong)Nx*(ulong)Ny;
			if((flags[nj] & TYPE_X) != 0u && (flags[nj] & TYPE_S) != 0u) {
				// neighbor in direction c_i is vehicle (TYPE_S|TYPE_X)
				wall_adjacent = true;
				// Phase 1: keep q_default. Phase 2: compute real q from ray-march / mesh intersection.
			}
		}
		if(wall_adjacent) {
			active_cells_host.push_back(n);
			active_q_host.push_back(q_per_dir);
		}
	}
	const uint N_active = (uint)active_cells_host.size();
	if(N_active == 0u) {
		print_warning("compute_bouzidi_cells_active: NO wall-adjacent fluid cells found (vehicle not voxelized?). Bouzidi remains disabled.");
		return;
	}
	print_info("Sparse Bouzidi: N_active = "+to_string(N_active)+" wall-adjacent fluid cells ("+to_string(100.0f*(float)N_active/(float)N,3u)+"% of total)");
	// Allocate device buffers per domain
	for(uint d=0u; d<get_D(); d++) {
		LBM_Domain* dom = lbm_domain[d];
		dom->bouzidi_N_active = N_active;
		dom->bouzidi_active_cells = Memory<ulong>(dom->device, (ulong)N_active);
		dom->bouzidi_q_data       = Memory<float>(dom->device, 19ull*(ulong)N_active);
		// Fill host data
		for(uint i=0u; i<N_active; i++) dom->bouzidi_active_cells[i] = active_cells_host[i];
		// SoA: q_data[dir*N_active + i_active]
		for(uint dir=0u; dir<19u; dir++) {
			for(uint i=0u; i<N_active; i++) dom->bouzidi_q_data[(ulong)dir*(ulong)N_active + (ulong)i] = active_q_host[i][dir];
		}
		dom->bouzidi_active_cells.write_to_device();
		dom->bouzidi_q_data.write_to_device();
		// (Re-)create kernel with proper range = N_active and buffers
		dom->kernel_apply_bouzidi_sparse = Kernel(dom->device, (ulong)N_active, "apply_bouzidi_sparse",
			dom->fi, dom->flags, dom->bouzidi_active_cells, dom->bouzidi_q_data, N_active, dom->t);
#ifdef WALL_SLIP_VEHICLE
		// Reuse same sparse active_cells list for wall_slip kernel (params: t, slip_factor, blend_strength)
		dom->kernel_apply_wall_slip = Kernel(dom->device, (ulong)N_active, "apply_wall_slip_vehicle",
			dom->fi, dom->rho, dom->u, dom->flags, dom->bouzidi_active_cells, N_active, dom->t, 1.0f, 0.0f);
#endif // WALL_SLIP_VEHICLE
	}
	bouzidi_enabled = true;
}
#endif // BOUZIDI_VEHICLE

void LBM::run(const ulong steps, const ulong total_steps) { // initializes the LBM simulation (copies data to device and runs initialize kernel), then runs LBM
	info.append(steps, total_steps, get_t()); // total_steps parameter is just for runtime estimation
	if(!initialized) {
		initialize();
		info.print_initialize(this); // only print setup info if the setup is new (run() was not called before)
#ifdef GRAPHICS
		camera.allow_rendering = true;
#endif // GRAPHICS
	}
	Clock clock;
	for(ulong i=1ull; i<=steps; i++) {
#if defined(INTERACTIVE_GRAPHICS)||defined(INTERACTIVE_GRAPHICS_ASCII)
		while(!key_P&&running) sleep(0.016);
		if(!running) break;
#endif // INTERACTIVE_GRAPHICS_ASCII || INTERACTIVE_GRAPHICS
		clock.start();
		do_time_step();
		info.update(clock.stop());
	}
	if(get_D()>1u) for(uint d=0u; d<get_D(); d++) lbm_domain[d]->finish_queue(); // wait for everything to finish (multi-GPU only)
}

void LBM::update_fields() { // update fields (rho, u, T) manually
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->enqueue_update_fields();
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->finish_queue();
}

void LBM::reset() { // reset simulation (takes effect in following run() call)
	initialized = false;
}

#ifdef FORCE_FIELD
void LBM::update_force_field() { // calculate forces from fluid on TYPE_S cells
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->enqueue_update_force_field();
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->finish_queue();
}
float3 LBM::object_center_of_mass(const uchar flag_marker) { // calculate center of mass of all cells flagged with flag_marker
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->enqueue_object_center_of_mass(flag_marker);
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->finish_queue();
	float3 object_com = float3(0.0f, 0.0f, 0.0f);
	ulong object_cells = 0ull;
	for(uint d=0u; d<get_D(); d++) {
		object_com += float3(lbm_domain[d]->object_sum.x[0], lbm_domain[d]->object_sum.y[0], lbm_domain[d]->object_sum.z[0]);
		object_cells += (ulong)as_uint(lbm_domain[d]->object_sum.w[0]);
	}
	return object_com/(float)object_cells;
}
float3 LBM::object_force(const uchar flag_marker) { // add up force for all cells flagged with flag_marker
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->enqueue_object_force(flag_marker);
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->finish_queue();
	float3 object_force = float3(0.0f, 0.0f, 0.0f);
	for(uint d=0u; d<get_D(); d++) object_force += float3(lbm_domain[d]->object_sum.x[0], lbm_domain[d]->object_sum.y[0], lbm_domain[d]->object_sum.z[0]);
	return object_force;
}
float3 LBM::object_torque(const float3& rotation_center, const uchar flag_marker) { // add up torque around specified rotation center for all cells flagged with flag_marker
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->enqueue_object_torque(rotation_center, flag_marker);
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->finish_queue();
	float3 object_torque = float3(0.0f, 0.0f, 0.0f);
	for(uint d=0u; d<get_D(); d++) object_torque += float3(lbm_domain[d]->object_sum.x[0], lbm_domain[d]->object_sum.y[0], lbm_domain[d]->object_sum.z[0]);
	return object_torque;
}
#endif // FORCE_FIELD

#ifdef MOVING_BOUNDARIES
void LBM::update_moving_boundaries() { // mark/unmark cells next to TYPE_S cells with velocity!=0 with TYPE_MS
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->enqueue_update_moving_boundaries();
	communicate_flags();
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->finish_queue();
#ifdef GRAPHICS
	camera.key_update = true; // to prevent flickering of flags in interactive graphics when camera is not moved
#endif // GRAPHICS
}
#endif // MOVING_BOUNDARIES

#if defined(PARTICLES)&&!defined(FORCE_FIELD)
void LBM::integrate_particles(const ulong steps, const ulong total_steps, const uint time_step_multiplicator) { // intgegrate passive tracer particles forward in time in stationary flow field
	info.append(steps, total_steps, get_t());
	Clock clock;
	for(ulong i=1ull; i<=steps; i+=(ulong)time_step_multiplicator) {
#if defined(INTERACTIVE_GRAPHICS)||defined(INTERACTIVE_GRAPHICS_ASCII)
		while(!key_P&&running) sleep(0.016);
		if(!running) break;
#endif // INTERACTIVE_GRAPHICS_ASCII || INTERACTIVE_GRAPHICS
		clock.start();
		for(uint d=0u; d<get_D(); d++) lbm_domain[d]->enqueue_integrate_particles(time_step_multiplicator);
		for(uint d=0u; d<get_D(); d++) lbm_domain[d]->finish_queue();
		for(uint d=0u; d<get_D(); d++) lbm_domain[d]->increment_time_step(time_step_multiplicator);
		info.update(clock.stop());
	}
}
#endif // PARTICLES&&!FORCE_FIELD

void LBM::write_status(const string& path) { // write LBM status report to a .txt file
	string status = "";
	status += "Grid Resolution = "+to_string(Nx)+" x "+to_string(Ny)+" x "+to_string(Nz)+" = "+to_string(get_N())+"\n";
	status += "Grid Domains = "+to_string(Dx)+" x "+to_string(Dy)+" x "+to_string(Dz)+" = "+to_string(get_D())+"\n";
	status += "LBM Type = D"+string(get_velocity_set()==9 ? "2" : "3")+"Q"+to_string(get_velocity_set())+" "+info.collision+"\n";
	status += "Memory Usage = CPU "+to_string(info.cpu_mem_required)+" MB, GPU "+to_string(get_D())+"x "+to_string(info.gpu_mem_required)+" MB\n";
	status += "Maximum Allocation Size = "+to_string((uint)(get_N()/(ulong)get_D()*(ulong)(get_velocity_set()*sizeof(fpxx))/1048576ull))+" MB\n";
	status += "Time Steps = "+to_string(get_t())+" / "+(info.steps==max_ulong ? "infinite" : to_string(info.steps))+"\n";
	status += "Runtime = "+print_time(info.runtime_total)+" (total) = "+print_time(info.runtime_lbm)+" (LBM) + "+print_time(info.runtime_total-info.runtime_lbm)+" (rendering and data evaluation)\n";
	status += "Average MLUPs/s = "+to_string(to_uint(1E-6*(double)get_N()*(double)get_t()/info.runtime_lbm))+"\n";
	status += "Kinematic Viscosity = "+to_string(get_nu())+"\n";
	status += "Relaxation Time = "+to_string(get_tau())+"\n";
	status += "Maximum Reynolds Number = "+to_string(get_Re_max())+"\n";
#ifdef VOLUME_FORCE
	status += "Volume Force = ("+to_string(get_fx())+", "+to_string(get_fy())+", "+to_string(get_fz())+")\n";
#endif // VOLUME_FORCE
#ifdef SURFACE
	status += "Surface Tension Coefficient = "+to_string(get_sigma())+"\n";
#endif // SURFACE
#ifdef TEMPERATURE
	status += "Thermal Diffusion Coefficient = "+to_string(get_alpha())+"\n";
	status += "Thermal Expansion Coefficient = "+to_string(get_beta())+"\n";
#endif // TEMPERATURE
	const string filename = default_filename(path, "status", ".txt", get_t());
	write_file(filename, status);
}

void LBM::voxelize_mesh_on_device(const Mesh* mesh, const uchar flag, const float3& rotation_center, const float3& linear_velocity, const float3& rotational_velocity) { // voxelize triangle mesh
	if(get_D()==1u) {
		lbm_domain[0]->voxelize_mesh_on_device(mesh, flag, rotation_center, linear_velocity, rotational_velocity); // if this crashes on Windows, create a TdrDelay 32-bit DWORD with decimal value 300 in Computer\HKEY_LOCAL_MACHINE\SYSTEM\CurrentControlSet\Control\GraphicsDrivers
	} else {
		parallel_for(get_D(), get_D(), [&](uint d) {
			lbm_domain[d]->voxelize_mesh_on_device(mesh, flag, rotation_center, linear_velocity, rotational_velocity);
		});
	}
#ifdef MOVING_BOUNDARIES
	if((flag&(TYPE_S|TYPE_E))==TYPE_S&&(length(linear_velocity)>0.0f||length(rotational_velocity)>0.0f)) update_moving_boundaries();
#endif // MOVING_BOUNDARIES
	if(!initialized) {
		flags.read_from_device();
		u.read_from_device();
	}
}
void LBM::unvoxelize_mesh_on_device(const Mesh* mesh, const uchar flag) { // remove voxelized triangle mesh from LBM grid by removing all flags in mesh bounding box (only required when bounding box size changes during re-voxelization)
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->enqueue_unvoxelize_mesh_on_device(mesh, flag);
	for(uint d=0u; d<get_D(); d++) lbm_domain[d]->finish_queue();
}
void LBM::write_mesh_to_vtk(const Mesh* mesh, const string& path, const bool convert_to_si_units) const { // write mesh to binary .vtk file
	const string filename = default_filename(path, "mesh", ".vtk", get_t());
	const string header_1 = "# vtk DataFile Version 3.0\nFluidX3D "+filename.substr(filename.rfind('/')+1)+"\nBINARY\nDATASET POLYDATA\nPOINTS "+to_string(3u*mesh->triangle_number)+" float\n";
	const string header_2 = "POLYGONS "+to_string(mesh->triangle_number)+" "+to_string(4u*mesh->triangle_number)+"\n";
	float* points = new float[9u*mesh->triangle_number];
	int* triangles = new int[4u*mesh->triangle_number];
	const float spacing = convert_to_si_units ? units.si_x(1.0f) : 1.0f;
	const float3 offset = center();
	parallel_for(mesh->triangle_number, [&](uint i) {
		points[9u*i   ] = reverse_bytes(spacing*(mesh->p0[i].x-offset.x));
		points[9u*i+1u] = reverse_bytes(spacing*(mesh->p0[i].y-offset.y));
		points[9u*i+2u] = reverse_bytes(spacing*(mesh->p0[i].z-offset.z));
		points[9u*i+3u] = reverse_bytes(spacing*(mesh->p1[i].x-offset.x));
		points[9u*i+4u] = reverse_bytes(spacing*(mesh->p1[i].y-offset.y));
		points[9u*i+5u] = reverse_bytes(spacing*(mesh->p1[i].z-offset.z));
		points[9u*i+6u] = reverse_bytes(spacing*(mesh->p2[i].x-offset.x));
		points[9u*i+7u] = reverse_bytes(spacing*(mesh->p2[i].y-offset.y));
		points[9u*i+8u] = reverse_bytes(spacing*(mesh->p2[i].z-offset.z));
		triangles[4u*i   ] = reverse_bytes(3); // 3 vertices per triangle
		triangles[4u*i+1u] = reverse_bytes(3*(int)i  ); // vertex 0
		triangles[4u*i+2u] = reverse_bytes(3*(int)i+1); // vertex 1
		triangles[4u*i+3u] = reverse_bytes(3*(int)i+2); // vertex 2
	});
	create_folder(filename);
	std::ofstream file(filename, std::ios::out|std::ios::binary);
	file.write(header_1.c_str(), header_1.length()); // write non-binary file header
	file.write((char*)points, 4u*9u*mesh->triangle_number); // write binary data
	file.write(header_2.c_str(), header_2.length()); // write non-binary file header
	file.write((char*)triangles, 4u*4u*mesh->triangle_number); // write binary data
	file.close();
	delete[] points;
	delete[] triangles;
	info.allow_printing.lock();
	print_info("File \""+filename+"\" saved.");
	info.allow_printing.unlock();
}
void LBM::voxelize_stl(const string& path, const float3& center, const float3x3& rotation, const float size, const uchar flag) { // voxelize triangle mesh
	const Mesh* mesh = read_stl(path, this->size(), center, rotation, size);
	flags.write_to_device();
	voxelize_mesh_on_device(mesh, flag);
	delete mesh;
	flags.read_from_device();
}
void LBM::voxelize_stl(const string& path, const float3x3& rotation, const float size, const uchar flag) { // read and voxelize binary .stl file (place in box center)
	voxelize_stl(path, center(), rotation, size, flag);
}
void LBM::voxelize_stl(const string& path, const float3& center, const float size, const uchar flag) { // read and voxelize binary .stl file (no rotation)
	voxelize_stl(path, center, float3x3(1.0f), size, flag);
}
void LBM::voxelize_stl(const string& path, const float size, const uchar flag) { // read and voxelize binary .stl file (place in box center, no rotation)
	voxelize_stl(path, center(), float3x3(1.0f), size, flag);
}

// Phase 5b-pre: Schwarz Multi-Resolution coupling — extract source-plane (u, rho), optionally
// smooth, apply at target-plane as TYPE_E BC. Same-resolution only in 5b-pre (src.cell_size==tgt.cell_size).
// Self-coupling (src == tgt, same plane) is the validation test — should be no-op invariant.
void LBM::couple_fields(LBM& tgt_sim, const PlaneSpec& src_plane, const PlaneSpec& tgt_plane, const CouplingOptions& opts) {
	if(src_plane.axis != tgt_plane.axis) { print_error("couple_fields: src and tgt plane axes differ"); return; }
	const uint axis = src_plane.axis;
	const uint a  = src_plane.extent_a; // src width  (in src cells)
	const uint b  = src_plane.extent_b; // src height (in src cells)
	const uint ta = tgt_plane.extent_a; // tgt width  (in tgt cells)
	const uint tb = tgt_plane.extent_b; // tgt height (in tgt cells)
	const ulong n_plane = (ulong)a * (ulong)b;
	const ulong n_tgt   = (ulong)ta * (ulong)tb;
	const bool needs_resample = (a != ta) || (b != tb); // Phase 5b-DR: bilinear up/downsample when src and tgt extents differ

	// Step 1: Sync src→host (triggers update_fields if UPDATE_FIELDS is not auto). PERF-D: caller may pre-sync and pass opts.sync_pcie=false.
	if(opts.sync_pcie) {
		u.read_from_device();
		rho.read_from_device();
	}

	// Step 2: Extract plane data (u_x, u_y, u_z, rho per cell)
	std::vector<float> plane_ux(n_plane), plane_uy(n_plane), plane_uz(n_plane), plane_rho(n_plane);
	const uint sx = src_plane.origin.x, sy = src_plane.origin.y, sz = src_plane.origin.z;
	for(uint i=0u; i<a; i++) {
		for(uint j=0u; j<b; j++) {
			uint cx = sx, cy = sy, cz = sz;
			if(axis == 0u) { cy = sy + i; cz = sz + j; }
			else if(axis == 1u) { cx = sx + i; cz = sz + j; }
			else /* axis == 2u */ { cx = sx + i; cy = sy + j; }
			const ulong n = (ulong)cx + (ulong)cy * (ulong)Nx + (ulong)cz * (ulong)Nx * (ulong)Ny;
			const ulong idx = (ulong)i + (ulong)j * (ulong)a;
			plane_ux[idx]  = u.x[n];
			plane_uy[idx]  = u.y[n];
			plane_uz[idx]  = u.z[n];
			plane_rho[idx] = rho[n];
		}
	}

	// Step 3: Optional 2D Gauss smoothing (3x3 or 5x5)
	if(opts.smooth_plane) {
		const int k = opts.smoothing_kernel_size; // 3 or 5
		const int hk = k / 2;
		std::vector<float> smooth_ux(n_plane), smooth_uy(n_plane), smooth_uz(n_plane), smooth_rho(n_plane);
		for(int i=0; i<(int)a; i++) {
			for(int j=0; j<(int)b; j++) {
				float wsum = 0.0f, ux_s=0.0f, uy_s=0.0f, uz_s=0.0f, rho_s=0.0f;
				for(int di=-hk; di<=hk; di++) {
					for(int dj=-hk; dj<=hk; dj++) {
						const int ii = i+di, jj = j+dj;
						if(ii<0 || ii>=(int)a || jj<0 || jj>=(int)b) continue;
						// Gauss weight: exp(-(di² + dj²) / (2 * (hk/2)²))
						const float r2 = (float)(di*di + dj*dj);
						const float sigma2 = 0.5f * (float)(hk * hk + 1);
						const float w = expf(-r2 / (2.0f * sigma2));
						const ulong nidx = (ulong)ii + (ulong)jj * (ulong)a;
						ux_s += w * plane_ux[nidx];
						uy_s += w * plane_uy[nidx];
						uz_s += w * plane_uz[nidx];
						rho_s += w * plane_rho[nidx];
						wsum += w;
					}
				}
				const ulong idx = (ulong)i + (ulong)j * (ulong)a;
				smooth_ux[idx] = ux_s / wsum;
				smooth_uy[idx] = uy_s / wsum;
				smooth_uz[idx] = uz_s / wsum;
				smooth_rho[idx] = rho_s / wsum;
			}
		}
		plane_ux  = std::move(smooth_ux);
		plane_uy  = std::move(smooth_uy);
		plane_uz  = std::move(smooth_uz);
		plane_rho = std::move(smooth_rho);
	}

	// Step 3b: Phase 5b-DR — bilinear resample from src grid (a×b) to tgt grid (ta×tb) if extents differ.
	// Up-sample when ta>a (Far→Near), down-sample when ta<a (Near→Far). Same array layout (idx = i + j*width).
	std::vector<float> tgt_ux, tgt_uy, tgt_uz, tgt_rho;
	if(!needs_resample) {
		tgt_ux  = std::move(plane_ux);
		tgt_uy  = std::move(plane_uy);
		tgt_uz  = std::move(plane_uz);
		tgt_rho = std::move(plane_rho);
	} else {
		tgt_ux.resize(n_tgt); tgt_uy.resize(n_tgt); tgt_uz.resize(n_tgt); tgt_rho.resize(n_tgt);
		const float scale_a = (ta > 1u) ? (float)(a - 1u) / (float)(ta - 1u) : 0.0f;
		const float scale_b = (tb > 1u) ? (float)(b - 1u) / (float)(tb - 1u) : 0.0f;
		// PERF-F (parallel bilinear 2026-05-14 evening): parallelize over n_tgt with parallel_for (OpenMP-like).
		// Each tgt cell is independent — thread-safe writes to distinct array indices.
		parallel_for(n_tgt, [&](ulong it_) {
			const uint it = (uint)(it_ % (ulong)ta);
			const uint jt = (uint)(it_ / (ulong)ta);
			const float xs = (float)it * scale_a;
			const float ys = (float)jt * scale_b;
			const uint i0 = (uint)floorf(xs);
			const uint j0 = (uint)floorf(ys);
			const uint i1 = (i0 + 1u < a) ? i0 + 1u : a - 1u;
			const uint j1 = (j0 + 1u < b) ? j0 + 1u : b - 1u;
			const float wx = xs - (float)i0;
			const float wy = ys - (float)j0;
			const ulong i00 = (ulong)i0 + (ulong)j0 * (ulong)a;
			const ulong i10 = (ulong)i1 + (ulong)j0 * (ulong)a;
			const ulong i01 = (ulong)i0 + (ulong)j1 * (ulong)a;
			const ulong i11 = (ulong)i1 + (ulong)j1 * (ulong)a;
			const float w00 = (1.0f-wx)*(1.0f-wy), w10 = wx*(1.0f-wy), w01 = (1.0f-wx)*wy, w11 = wx*wy;
			tgt_ux[it_]  = w00*plane_ux[i00]  + w10*plane_ux[i10]  + w01*plane_ux[i01]  + w11*plane_ux[i11];
			tgt_uy[it_]  = w00*plane_uy[i00]  + w10*plane_uy[i10]  + w01*plane_uy[i01]  + w11*plane_uy[i11];
			tgt_uz[it_]  = w00*plane_uz[i00]  + w10*plane_uz[i10]  + w01*plane_uz[i01]  + w11*plane_uz[i11];
			tgt_rho[it_] = w00*plane_rho[i00] + w10*plane_rho[i10] + w01*plane_rho[i01] + w11*plane_rho[i11];
		});
	}

	// Step 4: Sync tgt flags→host (read once; minimal cost). PERF-D: skip if caller pre-synced.
	if(opts.sync_pcie) tgt_sim.flags.read_from_device();
	// Step 4b: if α-blend active, also need current tgt u/rho to compute (1-α)*tgt + α*src. PERF-D: skip if caller pre-synced.
	const float alpha = (opts.alpha < 1.0f && opts.alpha > 0.0f) ? opts.alpha : 1.0f;
	const bool soft_blend = (alpha < 1.0f);
	if(opts.sync_pcie && soft_blend) {
		tgt_sim.u.read_from_device();
		tgt_sim.rho.read_from_device();
	}

	// Step 5: Write resampled plane data to target as TYPE_E (uses tgt extents, indexed against tgt grid).
	// PERF-F: parallelize over n_tgt — independent writes to distinct cell indices, thread-safe.
	const uint tx = tgt_plane.origin.x, ty = tgt_plane.origin.y, tz = tgt_plane.origin.z;
	parallel_for(n_tgt, [&](ulong idx_flat) {
		const uint i = (uint)(idx_flat % (ulong)ta);
		const uint j = (uint)(idx_flat / (ulong)ta);
		uint cx = tx, cy = ty, cz = tz;
		if(axis == 0u) { cy = ty + i; cz = tz + j; }
		else if(axis == 1u) { cx = tx + i; cz = tz + j; }
		else /* axis == 2u */ { cx = tx + i; cy = ty + j; }
		const ulong n = (ulong)cx + (ulong)cy * (ulong)tgt_sim.Nx + (ulong)cz * (ulong)tgt_sim.Nx * (ulong)tgt_sim.Ny;
		tgt_sim.flags[n]  = TYPE_E;
		if(soft_blend) {
			tgt_sim.u.x[n]  = (1.0f-alpha) * tgt_sim.u.x[n]  + alpha * tgt_ux[idx_flat];
			tgt_sim.u.y[n]  = (1.0f-alpha) * tgt_sim.u.y[n]  + alpha * tgt_uy[idx_flat];
			tgt_sim.u.z[n]  = (1.0f-alpha) * tgt_sim.u.z[n]  + alpha * tgt_uz[idx_flat];
			tgt_sim.rho[n]  = (1.0f-alpha) * tgt_sim.rho[n]  + alpha * tgt_rho[idx_flat];
		} else {
			tgt_sim.u.x[n]    = tgt_ux[idx_flat];
			tgt_sim.u.y[n]    = tgt_uy[idx_flat];
			tgt_sim.u.z[n]    = tgt_uz[idx_flat];
			tgt_sim.rho[n]    = tgt_rho[idx_flat];
		}
	});

	// Step 6: Sync host→target. PERF-D: skip if caller will write back after batch.
	if(opts.sync_pcie) {
		tgt_sim.flags.write_to_device();
		tgt_sim.u.write_to_device();
		tgt_sim.rho.write_to_device();
	}

	// Step 7: Optional diagnostics — operate on tgt-side data (post-resample) since plane_ux may have been moved-from
	if(opts.export_csv) {
		float ux_min = tgt_ux[0], ux_max = tgt_ux[0], ux_sum = 0.0f;
		float rho_min = tgt_rho[0], rho_max = tgt_rho[0], rho_sum = 0.0f;
		for(ulong k=0; k<n_tgt; k++) {
			ux_min = fminf(ux_min, tgt_ux[k]); ux_max = fmaxf(ux_max, tgt_ux[k]); ux_sum += tgt_ux[k];
			rho_min = fminf(rho_min, tgt_rho[k]); rho_max = fmaxf(rho_max, tgt_rho[k]); rho_sum += tgt_rho[k];
		}
		const string csv_path = get_exe_path() + "../bin/coupling_plane_t" + to_string(get_t()) + ".csv";
		std::ofstream f(csv_path, std::ios::app);
		if(f.tellp() == 0) f << "t,n_cells,ux_mean,ux_min,ux_max,rho_mean,rho_min,rho_max\n";
		f << get_t() << "," << n_tgt << "," << (ux_sum/n_tgt) << "," << ux_min << "," << ux_max << "," << (rho_sum/n_tgt) << "," << rho_min << "," << rho_max << "\n";
		f.close();
	}
}

#ifdef GRAPHICS
int* LBM::Graphics::draw_frame() {
#ifndef UPDATE_FIELDS
	if(visualization_modes&(VIS_FIELD|VIS_STREAMLINES|VIS_Q_CRITERION)) {
		for(uint d=0u; d<lbm->get_D(); d++) lbm->lbm_domain[d]->enqueue_update_fields(); // only call update_fields() if the time step has changed since the last rendered frame
	}
#endif // UPDATE_FIELDS
	if(key_1) { visualization_modes = (visualization_modes&~0b11)|(((visualization_modes&0b11)+1)%4); key_1 = false; }
	if(key_2) { visualization_modes ^= VIS_FIELD        ; key_2 = false; }
	if(key_3) { visualization_modes ^= VIS_STREAMLINES  ; key_3 = false; }
	if(key_4) { visualization_modes ^= VIS_Q_CRITERION  ; key_4 = false; }
	if(key_5) { visualization_modes ^= VIS_PHI_RASTERIZE; key_5 = false; }
	if(key_6) { visualization_modes ^= VIS_PHI_RAYTRACE ; key_6 = false; }
	if(key_7) { visualization_modes ^= VIS_PARTICLES    ; key_7 = false; }
	if(key_T) {
		slice_mode = (slice_mode+1)%8; key_T = false;
	}
	if(key_Z) {
#ifndef TEMPERATURE
		field_mode = (field_mode+1)%2; key_Z = false; // field_mode = { 0 (u), 1 (rho) }
#else // TEMPERATURE
		field_mode = (field_mode+1)%3; key_Z = false; // field_mode = { 0 (u), 1 (rho), 2 (T) }
#endif // TEMPERATURE
	}
	if(slice_mode==1u) {
		if(key_Q) { slice_x = clamp(slice_x-1, 0, (int)lbm->get_Nx()-1); key_Q = false; }
		if(key_E) { slice_x = clamp(slice_x+1, 0, (int)lbm->get_Nx()-1); key_E = false; }
	}
	if(slice_mode==2u) {
		if(key_Q) { slice_y = clamp(slice_y-1, 0, (int)lbm->get_Ny()-1); key_Q = false; }
		if(key_E) { slice_y = clamp(slice_y+1, 0, (int)lbm->get_Ny()-1); key_E = false; }
	}
	if(slice_mode==3u) {
		if(key_Q) { slice_z = clamp(slice_z-1, 0, (int)lbm->get_Nz()-1); key_Q = false; }
		if(key_E) { slice_z = clamp(slice_z+1, 0, (int)lbm->get_Nz()-1); key_E = false; }
	}
	const bool visualization_change = camera.key_update||last_visualization_modes!=visualization_modes||last_field_mode!=field_mode||last_slice_mode!=slice_mode||last_slice_x!=slice_x||last_slice_y!=slice_y||last_slice_z!=slice_z;
	camera.key_update = false;
	last_visualization_modes = visualization_modes;
	last_field_mode = field_mode;
	last_slice_mode = slice_mode;
	last_slice_x = slice_x;
	last_slice_y = slice_y;
	last_slice_z = slice_z;
	bool new_frame = true;
	for(uint d=0u; d<lbm->get_D(); d++) new_frame = new_frame && lbm->lbm_domain[d]->graphics.enqueue_draw_frame(visualization_modes, field_mode, slice_mode, slice_x, slice_y, slice_z, visualization_change);
	for(uint d=0u; d<lbm->get_D(); d++) lbm->lbm_domain[d]->finish_queue();
	int* bitmap = lbm->lbm_domain[0]->graphics.get_bitmap();
	int* zbuffer = lbm->lbm_domain[0]->graphics.get_zbuffer();
	for(uint d=1u; d<lbm->get_D()&&new_frame; d++) {
		const int* const bitmap_d = lbm->lbm_domain[d]->graphics.get_bitmap(); // each domain renders its own frame
		const int* const zbuffer_d = lbm->lbm_domain[d]->graphics.get_zbuffer();
		for(uint i=0u; i<camera.width*camera.height; i++) {
#ifndef GRAPHICS_TRANSPARENCY
			const int zdi = zbuffer_d[i];
			if(zdi>zbuffer[i]) {
				bitmap[i] = bitmap_d[i]; // overlay frames using their z-buffers
				zbuffer[i] = zdi;
			}
#else // GRAPHICS_TRANSPARENCY
			bitmap[i] = color_add(bitmap[i], bitmap_d[i]);
#endif // GRAPHICS_TRANSPARENCY
		}
	}
	camera.allow_labeling = new_frame; // only print new label on frame if a new frame has been rendered
	return bitmap;
}

void LBM::Graphics::set_camera_centered(const float rx, const float ry, const float fov, const float zoom) {
	camera.free = false;
	camera.rx = 0.5*pi+((double)rx*pi/180.0);
	camera.ry = pi-((double)ry*pi/180.0);
	camera.fov = clamp((float)fov, 1E-6f, 179.0f);
	camera.set_zoom(0.5f*(float)fmax(fmax(lbm->get_Nx(), lbm->get_Ny()), lbm->get_Nz())/zoom);
}
void LBM::Graphics::set_camera_free(const float3& p, const float rx, const float ry, const float fov) {
	camera.free = true;
	camera.rx = 0.5*pi+((double)rx*pi/180.0);
	camera.ry = pi-((double)ry*pi/180.0);
	camera.fov = clamp((float)fov, 1E-6f, 179.0f);
	camera.zoom = 1E16f;
	camera.pos = p;
}
bool LBM::Graphics::next_frame(const ulong total_time_steps, const float video_length_seconds) { // returns true once simulation time has progressed enough to render the next video frame for a 60fps video of specified length
	const uint new_frame = to_uint((float)lbm->get_t()/(float)total_time_steps*video_length_seconds*60.0f);
	if(new_frame!=last_exported_frame) {
		last_exported_frame = new_frame;
		return true;
	} else {
		return false;
	}
}
void LBM::Graphics::print_frame() { // preview current frame in console
#ifndef INTERACTIVE_GRAPHICS_ASCII
	camera.rendring_frame.lock(); // block rendering for other threads until finished
	camera.key_update = true; // force rendering new frame
	int* image_data = draw_frame(); // make sure the frame is fully rendered
	Image* image = new Image(camera.width, camera.height, image_data);
	info.allow_printing.lock();
	println();
	print_image(image);
	info.allow_printing.unlock();
	delete image;
	camera.rendring_frame.unlock();
#endif // INTERACTIVE_GRAPHICS_ASCII
}
void encode_image(Image* image, const string& filename, const string& extension, std::atomic_int* running_encoders) {
	if(extension==".png") write_png(filename, image);
	if(extension==".qoi") write_qoi(filename, image);
	if(extension==".bmp") write_bmp(filename, image);
	delete image; // delete image when done
	(*running_encoders)--;
}
void LBM::Graphics::write_frame(const string& path, const string& name, const string& extension, bool print_preview) { // save current frame as .png file (smallest file size, but slow)
	write_frame(0u, 0u, camera.width, camera.height, path, name, extension, print_preview);
}
void LBM::Graphics::write_frame(const uint x1, const uint y1, const uint x2, const uint y2, const string& path, const string& name, const string& extension, bool print_preview) { // save a cropped current frame with two corner points (x1,y1) and (x2,y2)
	camera.rendring_frame.lock(); // block rendering for other threads until finished
	camera.key_update = true; // force rendering new frame
	int* image_data = draw_frame(); // make sure the frame is fully rendered
	const string filename = default_filename(path, name, extension, lbm->get_t());
	const uint xa=max(min(x1, x2), 0u), xb=min(max(x1, x2), camera.width ); // sort coordinates if necessary
	const uint ya=max(min(y1, y2), 0u), yb=min(max(y1, y2), camera.height);
	Image* image = new Image(xb-xa, yb-ya); // create local copy of frame buffer
	for(uint y=0u; y<image->height(); y++) for(uint x=0u; x<image->width(); x++) image->set_color(x, y, image_data[camera.width*(ya+y)+(xa+x)]);
#ifndef INTERACTIVE_GRAPHICS_ASCII
	if(print_preview) {
		info.allow_printing.lock();
		println();
		print_image(image);
		print_info("Image \""+filename+"\" saved.");
		info.allow_printing.unlock();
	}
#endif // INTERACTIVE_GRAPHICS_ASCII
	running_encoders++;
	thread encoder(encode_image, image, filename, extension, &running_encoders); // the main bottleneck in rendering images to the hard disk is .png encoding, so encode image in new thread
	encoder.detach(); // detatch thread so it can run concurrently
	camera.rendring_frame.unlock();
}
void LBM::Graphics::write_frame_png(const string& path, bool print_preview) { // save current frame as .png file (smallest file size, but slow)
	write_frame(path, "image", ".png", print_preview);
}
void LBM::Graphics::write_frame_qoi(const string& path, bool print_preview) { // save current frame as .qoi file (small file size, fast)
	write_frame(path, "image", ".qoi", print_preview);
}
void LBM::Graphics::write_frame_bmp(const string& path, bool print_preview) { // save current frame as .bmp file (large file size, fast)
	write_frame(path, "image", ".bmp", print_preview);
}
void LBM::Graphics::write_frame_png(const uint x1, const uint y1, const uint x2, const uint y2, const string& path, bool print_preview) { // save current frame as .png file (smallest file size, but slow)
	write_frame(x1, y1, x2, y2, path, "image", ".png", print_preview);
}
void LBM::Graphics::write_frame_qoi(const uint x1, const uint y1, const uint x2, const uint y2, const string& path, bool print_preview) { // save current frame as .qoi file (small file size, fast)
	write_frame(x1, y1, x2, y2, path, "image", ".qoi", print_preview);
}
void LBM::Graphics::write_frame_bmp(const uint x1, const uint y1, const uint x2, const uint y2, const string& path, bool print_preview) { // save current frame as .bmp file (large file size, fast)
	write_frame(x1, y1, x2, y2, path, "image", ".bmp", print_preview);
}
#endif // GRAPHICS



void LBM_Domain::allocate_transfer(Device& device) { // allocate all memory for multi-device trqansfer
	ulong Amax = 0ull; // maximum domain side area of communicated directions
	if(Dx>1u) Amax = max(Amax, (ulong)Ny*(ulong)Nz); // Ax
	if(Dy>1u) Amax = max(Amax, (ulong)Nz*(ulong)Nx); // Ay
	if(Dz>1u) Amax = max(Amax, (ulong)Nx*(ulong)Ny); // Az

	transfer_buffer_p = Memory<char>(device, Amax, max(transfers*(uint)sizeof(fpxx), 17u), true, true, 0, false); // only allocate one set of transfer buffers in plus/minus directions, for all x/y/z transfers
	transfer_buffer_m = Memory<char>(device, Amax, max(transfers*(uint)sizeof(fpxx), 17u), true, true, 0, false); // these transfer buffers must not be zero-copy!

	kernel_transfer[enum_transfer_field::fi              ][0] = Kernel(device, 0ull, "transfer_extract_fi"              , 0u, t, transfer_buffer_p, transfer_buffer_m, fi);
	kernel_transfer[enum_transfer_field::fi              ][1] = Kernel(device, 0ull, "transfer__insert_fi"              , 0u, t, transfer_buffer_p, transfer_buffer_m, fi);
	kernel_transfer[enum_transfer_field::rho_u_flags     ][0] = Kernel(device, 0ull, "transfer_extract_rho_u_flags"     , 0u, t, transfer_buffer_p, transfer_buffer_m, rho, u, flags);
	kernel_transfer[enum_transfer_field::rho_u_flags     ][1] = Kernel(device, 0ull, "transfer__insert_rho_u_flags"     , 0u, t, transfer_buffer_p, transfer_buffer_m, rho, u, flags);
	kernel_transfer[enum_transfer_field::flags           ][0] = Kernel(device, 0ull, "transfer_extract_flags"           , 0u, t, transfer_buffer_p, transfer_buffer_m, flags);
	kernel_transfer[enum_transfer_field::flags           ][1] = Kernel(device, 0ull, "transfer__insert_flags"           , 0u, t, transfer_buffer_p, transfer_buffer_m, flags);
#ifdef FORCE_FIELD
	kernel_transfer[enum_transfer_field::F               ][0] = Kernel(device, 0ull, "transfer_extract_F"               , 0u, t, transfer_buffer_p, transfer_buffer_m, F);
	kernel_transfer[enum_transfer_field::F               ][1] = Kernel(device, 0ull, "transfer__insert_F"               , 0u, t, transfer_buffer_p, transfer_buffer_m, F);
#endif // FORCE_FIELD
#ifdef SURFACE
	kernel_transfer[enum_transfer_field::phi_massex_flags][0] = Kernel(device, 0ull, "transfer_extract_phi_massex_flags", 0u, t, transfer_buffer_p, transfer_buffer_m, phi, massex, flags);
	kernel_transfer[enum_transfer_field::phi_massex_flags][1] = Kernel(device, 0ull, "transfer__insert_phi_massex_flags", 0u, t, transfer_buffer_p, transfer_buffer_m, phi, massex, flags);
#endif // SURFACE
#ifdef TEMPERATURE
	kernel_transfer[enum_transfer_field::gi              ][0] = Kernel(device, 0ull, "transfer_extract_gi"              , 0u, t, transfer_buffer_p, transfer_buffer_m, gi);
	kernel_transfer[enum_transfer_field::gi              ][1] = Kernel(device, 0ull, "transfer__insert_gi"              , 0u, t, transfer_buffer_p, transfer_buffer_m, gi);
	kernel_transfer[enum_transfer_field::T               ][0] = Kernel(device, 0ull, "transfer_extract_T"               , 0u, t, transfer_buffer_p, transfer_buffer_m, T);
	kernel_transfer[enum_transfer_field::T               ][1] = Kernel(device, 0ull, "transfer__insert_T"               , 0u, t, transfer_buffer_p, transfer_buffer_m, T);
#endif // TEMPERATURE
}

ulong LBM_Domain::get_area(const uint direction) {
	const ulong A[3] = { (ulong)Ny*(ulong)Nz, (ulong)Nz*(ulong)Nx, (ulong)Nx*(ulong)Ny };
	return A[direction];
}
void LBM_Domain::enqueue_transfer_extract_field(Kernel& kernel_transfer_extract_field, const uint direction, const uint bytes_per_cell) {
	kernel_transfer_extract_field.set_ranges(get_area(direction)); // direction: x=0, y=1, z=2
	kernel_transfer_extract_field.set_parameters(0u, direction, get_t()).enqueue_run(); // selective in-VRAM copy
	transfer_buffer_p.enqueue_read_from_device(0ull, kernel_transfer_extract_field.range()*(ulong)bytes_per_cell); // PCIe copy (+)
	transfer_buffer_m.enqueue_read_from_device(0ull, kernel_transfer_extract_field.range()*(ulong)bytes_per_cell); // PCIe copy (-)
}
void LBM_Domain::enqueue_transfer_insert_field(Kernel& kernel_transfer_insert_field, const uint direction, const uint bytes_per_cell) {
	kernel_transfer_insert_field.set_ranges(get_area(direction)); // direction: x=0, y=1, z=2
	transfer_buffer_p.enqueue_write_to_device(0ull, kernel_transfer_insert_field.range()*(ulong)bytes_per_cell); // PCIe copy (+)
	transfer_buffer_m.enqueue_write_to_device(0ull, kernel_transfer_insert_field.range()*(ulong)bytes_per_cell); // PCIe copy (-)
	kernel_transfer_insert_field.set_parameters(0u, direction, get_t()).enqueue_run(); // selective in-VRAM copy
}
void LBM::communicate_field(const enum_transfer_field field, const uint bytes_per_cell) {
	if(Dx>1u) { // communicate in x-direction
		for(uint d=0u; d<get_D(); d++) lbm_domain[d]->enqueue_transfer_extract_field(lbm_domain[d]->kernel_transfer[field][0], 0u, bytes_per_cell); // selective in-VRAM copy (x) + PCIe copy
		for(uint d=0u; d<get_D(); d++) lbm_domain[d]->finish_queue(); // domain synchronization barrier
		for(uint d=0u; d<get_D(); d++) {
			const uint x=(d%(Dx*Dy))%Dx, y=(d%(Dx*Dy))/Dx, z=d/(Dx*Dy), dxp=((x+1u)%Dx)+(y+z*Dy)*Dx; // d = x+(y+z*Dy)*Dx
			lbm_domain[d]->transfer_buffer_p.exchange_host_buffer(lbm_domain[dxp]->transfer_buffer_m.exchange_host_buffer(lbm_domain[d]->transfer_buffer_p.data())); // CPU pointer swaps
		}
		for(uint d=0u; d<get_D(); d++) lbm_domain[d]-> enqueue_transfer_insert_field(lbm_domain[d]->kernel_transfer[field][1], 0u, bytes_per_cell); // PCIe copy + selective in-VRAM copy (x)
	}
	if(Dy>1u) { // communicate in y-direction
		for(uint d=0u; d<get_D(); d++) lbm_domain[d]->enqueue_transfer_extract_field(lbm_domain[d]->kernel_transfer[field][0], 1u, bytes_per_cell); // selective in-VRAM copy (y) + PCIe copy
		for(uint d=0u; d<get_D(); d++) lbm_domain[d]->finish_queue(); // domain synchronization barrier
		for(uint d=0u; d<get_D(); d++) {
			const uint x=(d%(Dx*Dy))%Dx, y=(d%(Dx*Dy))/Dx, z=d/(Dx*Dy), dyp=x+(((y+1u)%Dy)+z*Dy)*Dx; // d = x+(y+z*Dy)*Dx
			lbm_domain[d]->transfer_buffer_p.exchange_host_buffer(lbm_domain[dyp]->transfer_buffer_m.exchange_host_buffer(lbm_domain[d]->transfer_buffer_p.data())); // CPU pointer swaps
		}
		for(uint d=0u; d<get_D(); d++) lbm_domain[d]-> enqueue_transfer_insert_field(lbm_domain[d]->kernel_transfer[field][1], 1u, bytes_per_cell); // PCIe copy + selective in-VRAM copy (y)
	}
	if(Dz>1u) { // communicate in z-direction
		for(uint d=0u; d<get_D(); d++) lbm_domain[d]->enqueue_transfer_extract_field(lbm_domain[d]->kernel_transfer[field][0], 2u, bytes_per_cell); // selective in-VRAM copy (z) + PCIe copy
		for(uint d=0u; d<get_D(); d++) lbm_domain[d]->finish_queue(); // domain synchronization barrier
		for(uint d=0u; d<get_D(); d++) {
			const uint x=(d%(Dx*Dy))%Dx, y=(d%(Dx*Dy))/Dx, z=d/(Dx*Dy), dzp=x+(y+((z+1u)%Dz)*Dy)*Dx; // d = x+(y+z*Dy)*Dx
			lbm_domain[d]->transfer_buffer_p.exchange_host_buffer(lbm_domain[dzp]->transfer_buffer_m.exchange_host_buffer(lbm_domain[d]->transfer_buffer_p.data())); // CPU pointer swaps
		}
		for(uint d=0u; d<get_D(); d++) lbm_domain[d]-> enqueue_transfer_insert_field(lbm_domain[d]->kernel_transfer[field][1], 2u, bytes_per_cell); // PCIe copy + selective in-VRAM copy (z)
	}
}

void LBM::communicate_fi() {
	communicate_field(enum_transfer_field::fi, transfers*sizeof(fpxx));
}
void LBM::communicate_rho_u_flags() {
	communicate_field(enum_transfer_field::rho_u_flags, 17u);
}
void LBM::communicate_flags() {
	communicate_field(enum_transfer_field::flags, 1u);
}
#ifdef FORCE_FIELD
void LBM::communicate_F() {
	communicate_field(enum_transfer_field::F, 12u);
}
#endif // FORCE_FIELD
#ifdef SURFACE
void LBM::communicate_phi_massex_flags() {
	communicate_field(enum_transfer_field::phi_massex_flags, 9u);
}
#endif // SURFACE
#ifdef TEMPERATURE
void LBM::communicate_gi() {
	communicate_field(enum_transfer_field::gi, sizeof(fpxx));
}
void LBM::communicate_T() {
	communicate_field(enum_transfer_field::T, 4u);
}
#endif // TEMPERATURE
#ifdef PARTICLES
void LBM::communicate_particles() {
	if(get_D()>1u) {
		if(initialized) {
			for(uint d=0u; d<get_D(); d++) lbm_domain[d]->particles.enqueue_read_from_device();
			for(uint d=0u; d<get_D(); d++) lbm_domain[d]->finish_queue(); // domain synchronization barrier
			for(ulong n=0ull; n<lbm_domain[0]->particles.length(); n++) { // parallel_for(lbm_domain[0]->particles.length(), [&](ulong n) {
				for(uint d=1u; d<get_D(); d++) { // gather modified particle positions
					const float lbm_domain_d___particles_x_n_ = lbm_domain[d]->particles.x[n];
					if(as_uint(lbm_domain_d___particles_x_n_)!=0xFFFFFFFFu) { // particle was in domain d and has been modified
						lbm_domain[0]->particles.x[n] = lbm_domain_d___particles_x_n_;
						lbm_domain[0]->particles.y[n] = lbm_domain[d]->particles.y[n];
						lbm_domain[0]->particles.z[n] = lbm_domain[d]->particles.z[n];
						break; // particle can only be in one domain at a time, no need to check other domains once it has been found
					}
				}
			} // });
		}
		for(uint d=0u; d<get_D(); d++) { // broadcast unified particle positions, using pointer of lbm_domain[0] instead of memory copy
			float* lbm_domain_d_particles_data = lbm_domain[d]->particles.exchange_host_buffer(lbm_domain[0]->particles.data());
			lbm_domain[d]->particles.enqueue_write_to_device();
			lbm_domain[d]->particles.exchange_host_buffer(lbm_domain_d_particles_data);
		}
	}
}
#endif // PARTICLES