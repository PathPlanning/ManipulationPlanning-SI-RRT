# Info displayed in the add-on overview.
bl_info = {
    'name': "URDF Importer",
    'description': "",
    'author': "Victor-Louis De Gusseme",
    'version': (0, 1),
    'blender': (2, 80, 0),
    'location': "View3D > Tool Shelf",
    'wiki_url': 'https://github.com/Victorlouisdg/blender-urdf-importer',
    'support': "COMMUNITY",
    'category': "Import"
}

# Importing in this init file is a bit weird.
if "bpy" in locals():
    print("Force reloading the plugin.")
    import importlib

    importlib.reload(import_urdf)
    importlib.reload(import_urdf_operator)

else:
    from . import import_urdf, \
        import_urdf_operator

import bpy


# -------------------------------------------------------------------
#   Register & Unregister
# -------------------------------------------------------------------
def make_annotations(cls):
    """Converts class fields to annotations if running with Blender 2.8"""
    bl_props = {k: v for k, v in cls.__dict__.items() if isinstance(v, tuple)}
    if bl_props:
        if '__annotations__' not in cls.__dict__:
            setattr(cls, '__annotations__', {})
        annotations = cls.__dict__['__annotations__']
        for k, v in bl_props.items():
            annotations[k] = v
            delattr(cls, k)
    return cls

# All classes to register.
classes = (
    import_urdf_operator.URDF_OT_FilebrowserImporter,
)

def menu_func_import(self, context):
    self.layout.operator(import_urdf_operator.URDF_OT_FilebrowserImporter.bl_idname, text="URDF (.urdf)")

# Register all classes + the collection property for storing lightfields
def register():
    # Classes
    for cls in classes:
        make_annotations(cls)
        bpy.utils.register_class(cls)

    bpy.types.TOPBAR_MT_file_import.append(menu_func_import)


# Unregister all classes
# This is done in reverse to 'pop the register stack'.
def unregister():
    # Unregister classes
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)

    bpy.types.TOPBAR_MT_file_import.remove(menu_func_import)