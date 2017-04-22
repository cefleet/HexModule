#include "register_types.h"
#include "class_db.h"
#include "HexGrid.h"

void register_hex_types() {

        ClassDB::register_class<HexGrid>();
      //  ObjectTypeDB::register_type<Hex>();
}

void unregister_hex_types() {
   //nothing to do here
}
