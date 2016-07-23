#include "register_types.h"
#include "object_type_db.h"
#include "HexGrid.h"

void register_hex_types() {

        ObjectTypeDB::register_type<HexGrid>();
      //  ObjectTypeDB::register_type<Hex>();
}

void unregister_hex_types() {
   //nothing to do here
}
