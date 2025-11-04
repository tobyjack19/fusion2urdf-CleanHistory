# -*- coding: utf-8 -*-
"""
Created on Sun May 12 20:17:17 2019

@author: syuntoku
"""

import adsk, re
from xml.etree.ElementTree import Element, SubElement
from ..utils import utils

class Joint:
    def __init__(self, name, xyz, axis, parent, child, joint_type, upper_limit, lower_limit):
        """
        Attributes
        ----------
        name: str
            name of the joint
        type: str
            type of the joint(ex: rev)
        xyz: [x, y, z]
            coordinate of the joint
        axis: [x, y, z]
            coordinate of axis of the joint
        parent: str
            parent link
        child: str
            child link
        joint_xml: str
            generated xml describing about the joint
        tran_xml: str
            generated xml describing about the transmission
        """
        self.name = name
        self.type = joint_type
        self.xyz = xyz
        self.parent = parent
        self.child = child
        self.joint_xml = None
        self.tran_xml = None
        self.axis = axis  # for 'revolute' and 'continuous'
        self.upper_limit = upper_limit  # for 'revolute' and 'prismatic'
        self.lower_limit = lower_limit  # for 'revolute' and 'prismatic'
        
    def make_joint_xml(self):
        """
        Generate the joint_xml and hold it by self.joint_xml
        """
        joint = Element('joint')
        joint.attrib = {'name':self.name, 'type':self.type}
        
        origin = SubElement(joint, 'origin')
        origin.attrib = {'xyz':' '.join([str(_) for _ in self.xyz]), 'rpy':'0 0 0'}
        parent = SubElement(joint, 'parent')
        parent.attrib = {'link':self.parent}
        child = SubElement(joint, 'child')
        child.attrib = {'link':self.child}
        if self.type == 'revolute' or self.type == 'continuous' or self.type == 'prismatic':        
            axis = SubElement(joint, 'axis')
            axis.attrib = {'xyz':' '.join([str(_) for _ in self.axis])}
        if self.type == 'revolute' or self.type == 'prismatic':
            limit = SubElement(joint, 'limit')
            limit.attrib = {'upper': str(self.upper_limit), 'lower': str(self.lower_limit),
                            'effort': '100', 'velocity': '100'}
            
        self.joint_xml = "\n".join(utils.prettify(joint).split("\n")[1:])

    def make_transmission_xml(self):
        """
        Generate the tran_xml and hold it by self.tran_xml
        
        
        Notes
        -----------
        mechanicalTransmission: 1
        type: transmission interface/SimpleTransmission
        hardwareInterface: PositionJointInterface        
        """        
        
        tran = Element('transmission')
        tran.attrib = {'name':self.name + '_tran'}
        
        joint_type = SubElement(tran, 'type')
        joint_type.text = 'transmission_interface/SimpleTransmission'
        
        joint = SubElement(tran, 'joint')
        joint.attrib = {'name':self.name}
        hardwareInterface_joint = SubElement(joint, 'hardwareInterface')
        hardwareInterface_joint.text = 'hardware_interface/EffortJointInterface'
        
        actuator = SubElement(tran, 'actuator')
        actuator.attrib = {'name':self.name + '_actr'}
        hardwareInterface_actr = SubElement(actuator, 'hardwareInterface')
        hardwareInterface_actr.text = 'hardware_interface/EffortJointInterface'
        mechanicalReduction = SubElement(actuator, 'mechanicalReduction')
        mechanicalReduction.text = '1'
        
        self.tran_xml = "\n".join(utils.prettify(tran).split("\n")[1:])


def make_joints_dict(root, msg):
    """
    joints_dict holds parent, axis and xyz informatino of the joints
    
    
    Parameters
    ----------
    root: adsk.fusion.Design.cast(product)
        Root component
    msg: str
        Tell the status
        
    Returns
    ----------
    joints_dict: 
        {name: {type, axis, upper_limit, lower_limit, parent, child, xyz}}
    msg: str
        Tell the status
    """

    joint_type_list = [
    'fixed', 'revolute', 'prismatic', 'Cylinderical',
    'PinSlot', 'Planner', 'Ball']  # these are the names in urdf

    # temporary storage while we build the connectivity graph and compute positions
    joints_dict = {}
    temp = {}  # joint_name -> intermediate data including occurrence refs
    
    for joint in root.joints:
        joint_dict = {}
        joint_type = joint_type_list[joint.jointMotion.jointType]
        joint_dict['type'] = joint_type
        
        # swhich by the type of the joint
        joint_dict['axis'] = [0, 0, 0]
        joint_dict['upper_limit'] = 0.0
        joint_dict['lower_limit'] = 0.0
        
        # support  "Revolute", "Rigid" and "Slider"
        if joint_type == 'revolute':
            joint_dict['axis'] = [round(i, 6) for i in \
                joint.jointMotion.rotationAxisVector.asArray()] ## In Fusion, exported axis is normalized.
            max_enabled = joint.jointMotion.rotationLimits.isMaximumValueEnabled
            min_enabled = joint.jointMotion.rotationLimits.isMinimumValueEnabled            
            if max_enabled and min_enabled:  
                joint_dict['upper_limit'] = round(joint.jointMotion.rotationLimits.maximumValue, 6)
                joint_dict['lower_limit'] = round(joint.jointMotion.rotationLimits.minimumValue, 6)
            elif max_enabled and not min_enabled:
                msg = joint.name + 'is not set its lower limit. Please set it and try again.'
                break
            elif not max_enabled and min_enabled:
                msg = joint.name + 'is not set its upper limit. Please set it and try again.'
                break
            else:  # if there is no angle limit
                joint_dict['type'] = 'continuous'
                
        elif joint_type == 'prismatic':
            joint_dict['axis'] = [round(i, 6) for i in \
                joint.jointMotion.slideDirectionVector.asArray()]  # Also normalized
            max_enabled = joint.jointMotion.slideLimits.isMaximumValueEnabled
            min_enabled = joint.jointMotion.slideLimits.isMinimumValueEnabled            
            if max_enabled and min_enabled:  
                joint_dict['upper_limit'] = round(joint.jointMotion.slideLimits.maximumValue/100, 6)
                joint_dict['lower_limit'] = round(joint.jointMotion.slideLimits.minimumValue/100, 6)
            elif max_enabled and not min_enabled:
                msg = joint.name + 'is not set its lower limit. Please set it and try again.'
                break
            elif not max_enabled and min_enabled:
                msg = joint.name + 'is not set its upper limit. Please set it and try again.'
                break
        elif joint_type == 'fixed':
            pass
        
        # store the two connected link names (sanitized). We'll orient parent/child
        # later by traversing the kinematic tree starting from 'base_link'.
        if joint.occurrenceTwo.component.name == 'base_link':
            comp2_name = 'base_link'
        else:
            comp2_name = re.sub('[ :()]', '_', joint.occurrenceTwo.name)
        comp1_name = re.sub('[ :()]', '_', joint.occurrenceOne.name)
        joint_dict['comp1'] = comp1_name
        joint_dict['comp2'] = comp2_name

        # keep references to occurrences and joint object to compute joint world position
        temp[joint.name] = {
            'joint_obj': joint,
            'data': joint_dict
        }
        
        
        #There seem to be a problem with geometryOrOriginTwo. To calcualte the correct orogin of the generated stl files following approach was used.
        #https://forums.autodesk.com/t5/fusion-360-api-and-scripts/difference-of-geometryororiginone-and-geometryororiginonetwo/m-p/9837767
        #Thanks to Masaki Yamamoto!
        
        # Coordinate transformation by matrix
        # M: 4x4 transformation matrix
        # a: 3D vector
        def trans(M, a):
            ex = [M[0],M[4],M[8]]
            ey = [M[1],M[5],M[9]]
            ez = [M[2],M[6],M[10]]
            oo = [M[3],M[7],M[11]]
            b = [0, 0, 0]
            for i in range(3):
                b[i] = a[0]*ex[i]+a[1]*ey[i]+a[2]*ez[i]+oo[i]
            return(b)


        # Returns True if two arrays are element-wise equal within a tolerance
        def allclose(v1, v2, tol=1e-6):
            return( max([abs(a-b) for a,b in zip(v1, v2)]) < tol )

        try:
            xyz_from_one_to_joint = joint.geometryOrOriginOne.origin.asArray() # Relative Joint pos
            xyz_from_two_to_joint = joint.geometryOrOriginTwo.origin.asArray() # Relative Joint pos
            xyz_of_one            = joint.occurrenceOne.transform.translation.asArray() # Link origin
            xyz_of_two            = joint.occurrenceTwo.transform.translation.asArray() # Link origin
            M_two = joint.occurrenceTwo.transform.asArray() # Matrix as a 16 element array.

        # Compose joint position
            case1 = allclose(xyz_from_two_to_joint, xyz_from_one_to_joint)
            case2 = allclose(xyz_from_two_to_joint, xyz_of_one)
            if case1 or case2:
                xyz_of_joint = xyz_from_two_to_joint
            else:
                xyz_of_joint = trans(M_two, xyz_from_two_to_joint)


            joint_dict['xyz'] = [round(i / 100.0, 6) for i in xyz_of_joint]  # converted to meter

        except:
            try:
                if type(joint.geometryOrOriginTwo)==adsk.fusion.JointOrigin:
                    data = joint.geometryOrOriginTwo.geometry.origin.asArray()
                else:
                    data = joint.geometryOrOriginTwo.origin.asArray()
                joint_dict['xyz'] = [round(i / 100.0, 6) for i in data]  # converted to meter
            except:
                msg = joint.name + " doesn't have joint origin. Please set it and run again."
                break
        
        # don't insert into final dict yet

    # Build adjacency list: node -> list of (neighbor, joint_name)
    adj = {}
    for jname, info in temp.items():
        a = info['data']['comp1']
        b = info['data']['comp2']
        adj.setdefault(a, []).append((b, jname))
        adj.setdefault(b, []).append((a, jname))

    # BFS from base_link to orient the tree (parent -> child)
    from collections import deque
    root_node = 'base_link'
    visited = set()
    q = deque([root_node])
    visited.add(root_node)
    # track BFS level (distance) from base_link for each node
    levels = {root_node: 0}

    # helper to transform a local point by an occurrence transform array
    def transform_point(transform_array, local_point):
        # transform_array is 16-element array as returned by .asArray()
        ex = [transform_array[0], transform_array[4], transform_array[8]]
        ey = [transform_array[1], transform_array[5], transform_array[9]]
        ez = [transform_array[2], transform_array[6], transform_array[10]]
        oo = [transform_array[3], transform_array[7], transform_array[11]]
        res = [0, 0, 0]
        for i in range(3):
            res[i] = local_point[0]*ex[i] + local_point[1]*ey[i] + local_point[2]*ez[i] + oo[i]
        return res

    # compute joint world positions and assign oriented parent/child
    oriented = {}
    while q:
        node = q.popleft()
        for neigh, jname in adj.get(node, []):
            if neigh in visited:
                continue
            # orient joint from node (parent) to neigh (child)
            visited.add(neigh)
            q.append(neigh)
            levels[neigh] = levels.get(node, 0) + 1

            info = temp[jname]
            joint_obj = info['joint_obj']
            jdata = info['data']

            parent_name = node
            child_name = neigh

            # compute joint world position robustly using available geometry/origin
            world_pos = None
            # try occurrenceOne first
            try:
                if hasattr(joint_obj.geometryOrOriginOne, 'origin'):
                    local = joint_obj.geometryOrOriginOne.origin.asArray()
                else:
                    local = joint_obj.geometryOrOriginOne.geometry.origin.asArray()
                M1 = joint_obj.occurrenceOne.transform.asArray()
                world_pos = transform_point(M1, local)
            except Exception:
                world_pos = None

            # try occurrenceTwo if occurrenceOne didn't work or positions mismatch
            try:
                if hasattr(joint_obj.geometryOrOriginTwo, 'origin'):
                    local2 = joint_obj.geometryOrOriginTwo.origin.asArray()
                else:
                    local2 = joint_obj.geometryOrOriginTwo.geometry.origin.asArray()
                M2 = joint_obj.occurrenceTwo.transform.asArray()
                world_pos2 = transform_point(M2, local2)
                if world_pos is None:
                    world_pos = world_pos2
                else:
                    # if both exist but disagree, prefer the one closer to parent occurrence
                    # (choose the one that is numerically closer to parent's transform translation)
                    try:
                        parent_trans = joint_obj.occurrenceTwo.transform.translation.asArray() if parent_name == jdata['comp2'] else joint_obj.occurrenceOne.transform.translation.asArray()
                        # compute distances
                        d1 = sum([(a-b)**2 for a,b in zip(world_pos, parent_trans)])
                        d2 = sum([(a-b)**2 for a,b in zip(world_pos2, parent_trans)])
                        world_pos = world_pos if d1 <= d2 else world_pos2
                    except Exception:
                        # fallback: keep world_pos (occurrenceOne)
                        pass
            except Exception:
                pass

            if world_pos is None:
                # fallback: try joint geometryOrOriginTwo untranslated (as original code attempted)
                try:
                    if type(joint_obj.geometryOrOriginTwo)==adsk.fusion.JointOrigin:
                        data = joint_obj.geometryOrOriginTwo.geometry.origin.asArray()
                    else:
                        data = joint_obj.geometryOrOriginTwo.origin.asArray()
                    world_pos = data
                except Exception:
                    msg = joint_obj.name + " doesn't have joint origin. Please set it and run again."
                    return {}, msg

            # convert to meters
            world_pos_m = [round(i / 100.0, 6) for i in world_pos]

            # finalize joint dict
            final = {
                'type': jdata['type'],
                'axis': jdata.get('axis', [0,0,0]),
                'upper_limit': jdata.get('upper_limit', 0.0),
                'lower_limit': jdata.get('lower_limit', 0.0),
                'parent': parent_name,
                'child': child_name,
                'xyz': world_pos_m
            }
            joints_dict[jname] = final

    # For any joints not reached by BFS (disconnected components), fall back to original pairing
    for jname, info in temp.items():
        if jname in joints_dict:
            continue
        jdata = info['data']
        joint_obj = info['joint_obj']
        # default parent/child as original (comp2 -> comp1)
        try:
            comp1 = jdata['comp1']
            comp2 = jdata['comp2']
            # choose parent as the node closer to base_link (smaller level)
            if comp1 in levels and comp2 in levels:
                if levels[comp1] <= levels[comp2]:
                    parent_name = comp1
                    child_name = comp2
                else:
                    parent_name = comp2
                    child_name = comp1
            elif comp1 in levels:
                parent_name = comp1
                child_name = comp2
            elif comp2 in levels:
                parent_name = comp2
                child_name = comp1
            else:
                # neither endpoint reached in BFS; fall back to original assumption
                parent_name = comp2
                child_name = comp1
            # try to compute world pos similar to above
            world_pos = None
            try:
                if hasattr(joint_obj.geometryOrOriginTwo, 'origin'):
                    local = joint_obj.geometryOrOriginTwo.origin.asArray()
                else:
                    local = joint_obj.geometryOrOriginTwo.geometry.origin.asArray()
                M2 = joint_obj.occurrenceTwo.transform.asArray()
                world_pos = transform_point(M2, local)
            except Exception:
                pass
            if world_pos is None:
                try:
                    if type(joint_obj.geometryOrOriginTwo)==adsk.fusion.JointOrigin:
                        data = joint_obj.geometryOrOriginTwo.geometry.origin.asArray()
                    else:
                        data = joint_obj.geometryOrOriginTwo.origin.asArray()
                    world_pos = data
                except Exception:
                    msg = joint_obj.name + " doesn't have joint origin. Please set it and run again."
                    return {}, msg
            world_pos_m = [round(i / 100.0, 6) for i in world_pos]
            final = {
                'type': jdata['type'],
                'axis': jdata.get('axis', [0,0,0]),
                'upper_limit': jdata.get('upper_limit', 0.0),
                'lower_limit': jdata.get('lower_limit', 0.0),
                'parent': parent_name,
                'child': child_name,
                'xyz': world_pos_m
            }
            joints_dict[jname] = final
        except Exception:
            # ignore and continue
            pass

    return joints_dict, msg
