/**************************************************************************/
/*  register_types.cpp                                                    */
/**************************************************************************/

#include "register_types.h"

#include "proxy_cloth.h"

#include "core/object/class_db.h"

void initialize_proxy_cloth_module(ModuleInitializationLevel p_level) {
	if (p_level == MODULE_INITIALIZATION_LEVEL_SCENE) {
		GDREGISTER_CLASS(ProxyCloth);
	}
}

void uninitialize_proxy_cloth_module(ModuleInitializationLevel p_level) {
}
