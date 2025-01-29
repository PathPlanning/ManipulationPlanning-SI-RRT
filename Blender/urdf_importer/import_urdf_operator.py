from . import import_urdf
import bpy
from bpy_extras.io_utils import ImportHelper 
from bpy.types import Operator
from bpy.props import StringProperty


class URDF_OT_FilebrowserImporter(Operator, ImportHelper): 
    bl_idname = "import.urdf_filebrowser" 
    bl_label = "Import URDF" 

    filter_glob: StringProperty(
        default="*.urdf",
        options={'HIDDEN'},
        maxlen=255,  # Max internal buffer length, longer would be clamped.
    )

    def execute(self, context): 
        import_urdf.import_urdf(self.filepath)
        return {'FINISHED'}
    
