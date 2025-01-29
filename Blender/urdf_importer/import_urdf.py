import os
import glob
import xml.etree.ElementTree as ET
import bpy
import numpy as np
from mathutils import Vector
import math
import mathutils
import bmesh
def find_rootlinks(joints):
    """Return all links that don't occur as child in any joint"""
    parents = []
    children = []
    for joint in joints:
        parents.append(joint.find('parent').attrib['link'])
        children.append(joint.find('child').attrib['link'])

    rootlinks = list(set(parents) - set(children))
    return rootlinks


def find_childjoints(joints, link):
    """Returns all joints that contain the link as parent"""
    childjoints = []
    for joint in joints:
        if joint.find('parent').attrib['link'] == link:
            childjoints.append(joint)
    return childjoints


def select_only(blender_object):
    """Selects and actives a Blender object and deselects all others"""
    bpy.ops.object.select_all(action='DESELECT')
    bpy.context.view_layer.objects.active = blender_object
    blender_object.select_set(True)


def add_next_empty(empty, joint):
    """Duplicates the empty and applies the transform specified in the joint"""
    select_only(empty)
    
    bpy.ops.object.duplicate()
    new_empty = bpy.context.active_object
    new_empty.name = 'TF_' + joint.attrib['name']

    origin = joint.find('origin')
    if origin is not None:
        if 'xyz' in origin.attrib:
            translation = [float(s) for s in origin.attrib['xyz'].split()]
            # bpy.ops.transform.translate(value=translation, orient_type='LOCAL')
            mat_trans = mathutils.Matrix.Translation(translation)
            new_empty.matrix_world =  new_empty.matrix_world @mat_trans 
        else:
            raise ValueError("awtoewngdsov")

        
        
        if 'rpy' in origin.attrib:
            roll, pitch, yaw = [float(s) for s in origin.attrib['rpy'].split()]    
            R = mathutils.Euler((roll, pitch, yaw)).to_matrix().to_4x4()
            new_empty.matrix_world = new_empty.matrix_world @R
        else:
            raise ValueError("awtoewngdsov")

    bpy.context.view_layer.update()
    return new_empty


def parse_mesh_filename(mesh_filename):
    """This function will return the mesh path if it can be found, else throw an error"""
    if os.path.exists(mesh_filename):
        return mesh_filename
    
    if 'package://' in mesh_filename:
        ros_package_paths = os.environ.get('ROS_PACKAGE_PATH')
        if ros_package_paths is None:
            error_msg = (
                'Your urdf file references a mesh file from a ROS package: \n'
                f'{mesh_filename}\n'
                'However, the ROS_PACKAGE_PATH environment variable is not set ' 
                'so we cannot find it.'
            )
            print(error_msg)
            # TODO throw error
            
        ros_package_paths = ros_package_paths.split(':')

        print(ros_package_paths)

        for ros_package_path in ros_package_paths:
            filepath_package = mesh_filename.replace('package://', '')
            
            filepath_split = filepath_package.split('/')
            package_name = filepath_split[0]
            filepath_in_package = os.path.join(*filepath_split[1:])

            for package_path in glob.glob(ros_package_path + '/**' + package_name):
                filepath = os.path.join(package_path, filepath_in_package)
                if os.path.exists(filepath):
                    return filepath

            for package_path in glob.glob(ros_package_path + '/**/' + package_name):
                filepath = os.path.join(package_path, filepath_in_package)
                if os.path.exists(filepath):
                    return filepath

    # global filename_abs
    print('filename_abs ',filename_abs)
    if os.path.exists(os.path.join(os.path.dirname(filename_abs), mesh_filename)):
        return os.path.join(os.path.dirname(filename_abs), mesh_filename)
    assert(False)
    print('Cant find the mesh file :(')
    # TODO if we get here, throw an error


def load_mesh(mesh):
    mesh_filename = mesh.attrib['filename']
    print(mesh_filename)
    mesh_path = parse_mesh_filename(mesh_filename)
    print(mesh_path)
    file_extension = mesh_filename.split('.')[-1]
    objects = None
    print("file_extension:",file_extension)
    if file_extension == "stl":
        bpy.ops.import_mesh.stl(filepath=mesh_path)
        objects = [bpy.context.object]
    elif file_extension == "dae":
        bpy.ops.wm.collada_import(filepath=mesh_path)
        objects = bpy.context.selected_objects
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    objects_to_delete = [o for o in bpy.context.scene.objects if o.type in ('CAMERA', 'LIGHT')]
    with bpy.context.temp_override(selected_objects=objects_to_delete): 
        bpy.ops.object.delete()
        
    assert(objects is not None)
    return objects


def load_geometry(visual):
    geometry = visual.find('geometry')

    mesh = geometry.find('mesh')
    if mesh is not None:
        return load_mesh(mesh)

    cylinder = geometry.find('cylinder')
    if cylinder is not None:
        length = float(cylinder.attrib['length'])
        radius = float(cylinder.attrib['radius'])
        bpy.ops.mesh.primitive_cylinder_add(vertices=64, radius=radius, depth=length)
        return [bpy.context.active_object]

    capsule = geometry.find('capsule')
    if capsule is not None:
        length = float(capsule.attrib['length'])
        radius = float(capsule.attrib['radius'])
        
        bm = bmesh.new()
        bmesh.ops.create_uvsphere(bm, u_segments=64, v_segments=64, radius=radius)
        delta_Z = length/2
        bm.verts.ensure_lookup_table()
        for vert in bm.verts:
            if vert.co[2] < 0:
                vert.co[2] -= delta_Z
            elif vert.co[2] > 0:
                vert.co[2] += delta_Z

        name = 'Capsule'
        mesh = bpy.data.meshes.new(name)
        bm.to_mesh(mesh)
        mesh.update()
        bm.free()

        object = bpy.data.objects.new(name, mesh)
        bpy.context.scene.collection.objects.link(object)

        return [object]
    
    box = geometry.find('box')
    if box is not None:
        x, y, z = [float(s) for s in box.attrib['size'].split()]
        bpy.ops.mesh.primitive_cube_add()
        cube = bpy.context.active_object
        cube.dimensions = Vector((x, y, z))
        bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
        return [cube]

    sphere = geometry.find('sphere')
    if sphere is not None:
        radius = float(sphere.attrib['radius'])
        bpy.ops.mesh.primitive_ico_sphere_add(subdivisions=6
                                              , radius=radius)
        return [bpy.context.active_object]
        
    return []


def add_revolute_joint_bone(armature, joint, empty, parent_bone_name):
    axis = Vector([float(s) for s in joint.find('axis').attrib['xyz'].split()])
    axis_world = empty.matrix_world.to_3x3() @ axis

    select_only(armature)
    bpy.ops.object.mode_set(mode='EDIT')
    eb = armature.data.edit_bones.new(joint.attrib['name'])
    eb.head = empty.location
    eb.tail = empty.location + axis_world / 10
    eb.parent = armature.data.edit_bones[parent_bone_name]
    bone_name = eb.name

    bpy.ops.object.mode_set(mode='POSE')

    posebone = armature.pose.bones[bone_name]

    posebone.rotation_mode = 'XYZ'
    posebone.lock_rotation[0] = True
    posebone.lock_rotation[1] = False
    posebone.lock_rotation[2] = True

    posebone.lock_ik_x = True
    posebone.lock_ik_y = False
    posebone.lock_ik_z = True
    posebone.bone.use_deform = False
    posebone.lock_scale[0] = True
    posebone.lock_scale[1] = True
    posebone.lock_scale[2] = True
    posebone.lock_location[0] = True
    posebone.lock_location[1] = True
    posebone.lock_location[2] = True
    print(joint)
    print(joint.attrib)
    print(joint.find('limit'))
    
    if joint.find('limit') is not None:
        print("joint.find('limit').attrib ",joint.find('limit').attrib)
        rotation_constrain = posebone.constraints.new("LIMIT_ROTATION")
        rotation_constrain.use_transform_limit = True
        rotation_constrain.use_limit_y = True
        rotation_constrain.min_y = float(joint.find('limit').attrib['lower'])
        rotation_constrain.max_y = float(joint.find('limit').attrib['upper'])

    
    bpy.ops.object.mode_set(mode='OBJECT')
    return bone_name


def position_link_objects(visual, objects, empty, joint_name):
    for i, object in enumerate(objects):
        select_only(object)
        print(bpy.context.selected_objects)
        object.matrix_world = empty.matrix_world
        object.name = 'DEFORM__' + joint_name + '__' + str(i)
        
        bpy.context.scene.cursor.matrix = empty.matrix_world

        origin = visual.find('origin')
        if origin is not None:
            if 'xyz' in origin.attrib:
                translation = [float(s) for s in origin.attrib['xyz'].split()]
                # bpy.ops.transform.translate(value=translation, orient_type='LOCAL')
                mat_trans = mathutils.Matrix.Translation(translation)
                object.matrix_world =  object.matrix_world @mat_trans 
            else:
                raise ValueError("awtoewngdsov")
         
            
            if 'rpy' in origin.attrib:
                roll, pitch, yaw = [float(s) for s in origin.attrib['rpy'].split()]    
                R = mathutils.Euler((roll, pitch, yaw)).to_matrix().to_4x4()
                object.matrix_world = object.matrix_world @R
            else:
                raise ValueError("awtoewngdsov")


            
        bpy.context.view_layer.update()


def add_childjoints(armature, joints, links, link, empty, parent_bone_name):
    childjoints = find_childjoints(joints, link)
    for childjoint in childjoints:
        new_empty = add_next_empty(empty, childjoint)
        
        bone_name = parent_bone_name
        
        if childjoint.attrib['type'] == 'revolute':
            bone_name = add_revolute_joint_bone(armature, childjoint, new_empty, parent_bone_name)
            
        
        # Find the childlink xml object
        childlink_name = childjoint.find('child').attrib['link']
        for childlink in links:
            if childlink.attrib['name'] == childlink_name:
                break
        
        visual = childlink.find('visual')
        if visual is not None:
            objects = load_geometry(visual)
            position_link_objects(visual, objects, new_empty, bone_name)
            for object in objects:
                object.parent = armature
                object.parent_bone = bone_name
                object.parent_type = 'BONE'
                m = object.matrix_world.copy()
                object.matrix_local @= m
        add_childjoints(armature, joints, links, childlink_name, new_empty, bone_name)


def assign_vertices_to_group(object, groupname):
    select_only(object)
    print(f'{object.vertex_groups=}')
    group = object.vertex_groups[groupname]
    indices = [v.index for v in bpy.context.selected_objects[0].data.vertices]
    group.add(indices, 1.0, type='ADD')


def import_urdf(filepath):
    
    if not os.path.exists(filepath):
        print('File does not exist')
    global filename_abs
    filename_abs = filepath
    tree = ET.parse(filepath)
    xml_root = tree.getroot()

    links = xml_root.findall('link')
    joints = xml_root.findall('joint')

    if joints:
        rootlinks = find_rootlinks(joints)
    else:
        rootlinks = [link.attrib['name'] for link in links]

    print("rootlinks:", rootlinks)
    if len(rootlinks)>1:
        raise NotImplemented("Need to reimplement import with several roots")
    for rootlink in rootlinks:
               
        #Place root empty arrows and armature 
        bpy.ops.object.empty_add(type='ARROWS', align='WORLD', location=(0, 0, 0), scale=(1, 1, 1))
        bpy.context.object.empty_display_size = 0.2
        empty = bpy.context.active_object
        empty.name = 'TF_' + rootlink
        
        bpy.ops.object.armature_add(radius=0.05, enter_editmode=False, align='WORLD', location=(0, 0, 0), scale=(1, 1, 1))    
        armature = bpy.context.active_object
        

        bone_name = 'root'
        bpy.context.active_bone.name = bone_name

        select_only(armature)
        bpy.ops.object.mode_set(mode='POSE')
        armature.pose.bones[bone_name].lock_ik_x = True
        armature.pose.bones[bone_name].lock_ik_y = True
        armature.pose.bones[bone_name].lock_ik_z = True
        armature.pose.bones[bone_name].lock_scale[0] = True
        armature.pose.bones[bone_name].lock_scale[1] = True
        armature.pose.bones[bone_name].lock_scale[2] = True
        armature.pose.bones[bone_name].lock_rotation[0] = True
        armature.pose.bones[bone_name].lock_rotation[1] = False
        armature.pose.bones[bone_name].lock_rotation[2] = True
        armature.pose.bones[bone_name].lock_location[0] = True
        armature.pose.bones[bone_name].lock_location[1] = True
        armature.pose.bones[bone_name].lock_location[2] = True
        

        armature.pose.bones[bone_name].bone.use_deform = False
        bpy.ops.object.mode_set(mode='OBJECT')
        

    
        for link in links:
            if link.attrib['name'] == rootlink:
                break

        visual = link.find('visual')
        if visual is not None:
            objects = load_geometry(visual)
            for object in objects:
                # object.data.use_auto_smooth = True
                select_only(object)
                object.parent = armature
                object.parent_bone = bone_name
                object.parent_type = 'BONE'
                m = object.matrix_world.copy()
                object.matrix_local @= m


        
        add_childjoints(armature, joints, links, rootlink, empty, bone_name)

        ## Skinning
        select_only(armature)

      

        # # Delete the empties
        # bpy.ops.object.select_all(action='DESELECT') 
        # for object in bpy.data.objects:
        #     if 'TF_' in object.name:
        #         object.select_set(True)
        # bpy.ops.object.delete() 


if __name__ == '__main__':
    filepath = '/home/idlab185/ur10.urdf'
    import_urdf(filepath)
