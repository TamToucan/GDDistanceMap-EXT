#include "../../distancemap/src/gdextension_registration.hpp"

#include <gdextension_interface.h>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/godot.hpp>

#include <GDDistanceMap.hpp>

using namespace godot;

void initialize_libgddistanceMap(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_CORE) {
		return;
	}
	ClassDB::register_class<GDDistanceMap>();
	std::cout << "################# REGISTER GDDistanceMap" << std::endl;
}

void uninitialize_libgddistanceMap(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_CORE) {
		return;
	}
}

extern "C" {

GDExtensionBool GDE_EXPORT libgddistanceMap_init(GDExtensionInterfaceGetProcAddress p_get_proc_address,
										       const GDExtensionClassLibraryPtr p_library,
		                                       GDExtensionInitialization *r_initialization) {
	godot::GDExtensionBinding::InitObject init_obj(p_get_proc_address, p_library, r_initialization);

	init_obj.register_initializer(initialize_libgddistanceMap);
	init_obj.register_terminator(uninitialize_libgddistanceMap);
	init_obj.set_minimum_library_initialization_level(MODULE_INITIALIZATION_LEVEL_CORE);

	return init_obj.init();
}

}
