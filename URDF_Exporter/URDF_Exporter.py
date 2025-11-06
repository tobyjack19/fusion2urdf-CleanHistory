#Author-syuntoku14
#Description-Generate URDF file from Fusion 360

import adsk, adsk.core, adsk.fusion, traceback
import os
import re
import sys
from .utils import utils
from .core import Link, Joint, Write

"""
# length unit is 'cm' and inertial unit is 'kg/cm^2'
# If there is no 'body' in the root component, maybe the corrdinates are wrong.
"""

# joint effort: 100
# joint velocity: 100
# supports "Revolute", "Rigid" and "Slider" joint types

# I'm not sure how prismatic joint acts if there is no limit in fusion model

def run(context):
    ui = None
    success_msg = 'Successfully create URDF file'
    msg = success_msg
    
    try:
        # --------------------
        # initialize
        app = adsk.core.Application.get()
        ui = app.userInterface
        dlg = None
        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        title = 'Fusion2URDF'
        if not design:
            ui.messageBox('No active Fusion design', title)
            return

        # Detailed export log accumulator
        logs = []
        def log(message):
            try:
                logs.append(str(message))
            except Exception:
                pass

        root = design.rootComponent  # root component 
        components = design.allComponents

        # set the names        
        robot_name = root.name.split()[0]
        package_name = robot_name + '_description'

        # Set up a progress dialog for clear user feedback
        try:
            dlg = ui.createProgressDialog()
            dlg.isBackgroundTranslucency = False
            dlg.cancelButtonText = 'Cancel'
            # 7 main steps below
            dlg.show(title, 'Step %v of %m: %p%', 0, 7, 1)
        except Exception:
            dlg = None

        def _tick(step_message):
            try:
                if dlg:
                    dlg.message = step_message
                    dlg.setProgressValue(dlg.progressValue + 1)
                # mirror into log
                log(f"[step] {step_message}")
            except Exception:
                pass

        def _check_cancel():
            try:
                return dlg and dlg.wasCancelled
            except Exception:
                return False

        _tick('Selecting target folder...')
        # Ask user for target folder (base directory)
        base_dir = utils.file_dialog(ui)
        if base_dir == False:
            try:
                if dlg: dlg.hide()
            except Exception:
                pass
            ui.messageBox('Fusion2URDF was canceled', title)
            return 0
        if _check_cancel():
            if dlg: dlg.hide()
            ui.messageBox('Fusion2URDF was canceled', title)
            return 0

        # If a package with the same name already exists in the selected folder,
        # append a version suffix _vN where N is 1 higher than the highest
        # existing version number found. Existing names handled are:
        #   package_name
        #   package_name_v1, package_name_v2, ...
        try:
            existing_versions = []
            for name in os.listdir(base_dir):
                if name == package_name:
                    existing_versions.append(0)
                else:
                    m = re.match(re.escape(package_name) + r'_v(\d+)$', name)
                    if m:
                        try:
                            existing_versions.append(int(m.group(1)))
                        except ValueError:
                            pass
            if existing_versions:
                new_ver = max(existing_versions) + 1
                package_name = f"{package_name}_v{new_ver}"
        except Exception:
            # If anything goes wrong (permissions, etc.), fall back to original name
            pass

        # Final save directory is the selected folder + package_name
        save_dir = os.path.join(base_dir, package_name)
        try:
            os.mkdir(save_dir)
        except:
            pass    

        package_dir = os.path.abspath(os.path.dirname(__file__)) + '/package/'
        
        # --------------------
        # set dictionaries
        _tick('Building joints...')
        # Generate joints_dict. All joints are related to root. 
        try:
            joints_dict, msg = Joint.make_joints_dict(root, msg)
        except Exception:
            if dlg: dlg.hide()
            ui.messageBox('Failed while creating joints:\n{}'.format(traceback.format_exc()), title)
            return 0
        if msg != success_msg:
            if dlg: dlg.hide()
            ui.messageBox(msg, title)
            return 0
        if not joints_dict:
            if dlg: dlg.hide()
            ui.messageBox('No joints were found. Please check your Fusion design and try again.', title)
            return 0
        if _check_cancel():
            if dlg: dlg.hide()
            ui.messageBox('Fusion2URDF was canceled', title)
            return 0

        try:
            inertial_dict, msg = Link.make_inertial_dict(root, msg)
        except Exception:
            if dlg: dlg.hide()
            ui.messageBox('Failed while computing inertials:\n{}'.format(traceback.format_exc()), title)
            return 0
        
        # Link positions dict
        links_xyz_dict = {}

        # --------------------
        # Parse mimics embedded in joint names: "<follower>-Link-<leader>:<ratio>[:<offset>]"
        def _base_name(name):
            try:
                return name.split('-Link-')[0]
            except Exception:
                return name

        def _annotate_mimics_from_names(joints_dict):
            processed = 0
            unmatched = 0
            # Build a mapping from base name to actual joint key
            base_to_key = {}
            for k in joints_dict.keys():
                base_to_key[_base_name(k)] = k
            # Regex for pattern
            patt = re.compile(r'^(?P<base>.+?)-Link-(?P<leader>[^:]+):(?P<mult>[+-]?(?:\d+\.\d*|\d*\.\d+|\d+)(?:[eE][+-]?\d+)?)(?::(?P<offset>[+-]?(?:\d+\.\d*|\d*\.\d+|\d+)(?:[eE][+-]?\d+)?))?$', re.UNICODE)

            for j in list(joints_dict.keys()):
                m = patt.match(j)
                # Always set output_name to the base part (clean name for URDF)
                joints_dict[j]['output_name'] = _base_name(j)
                if not m:
                    continue
                leader_raw = m.group('leader')
                mult = float(m.group('mult')) if m.group('mult') is not None else 1.0
                off = m.group('offset')
                offset = float(off) if off is not None else 0.0
                # Resolve leader
                leader_key = None
                if leader_raw in joints_dict:
                    leader_key = leader_raw
                elif leader_raw in base_to_key:
                    leader_key = base_to_key[leader_raw]
                else:
                    # Try relaxed: match by base-insensitive
                    for k in joints_dict.keys():
                        if _base_name(k).lower() == leader_raw.lower():
                            leader_key = k
                            break
                if not leader_key:
                    unmatched += 1
                    log(f"[name-link] follower={j}: leader '{leader_raw}' not found -> skipped")
                    continue
                # Only set mimic for supported types
                if joints_dict[j]['type'] in ('revolute', 'continuous', 'prismatic'):
                    if joints_dict[j].get('mimic') is None:
                        joints_dict[j]['mimic'] = {
                            'joint': joints_dict[leader_key].get('output_name', _base_name(leader_key)),
                            'multiplier': mult,
                            'offset': offset
                        }
                        processed += 1
                        log(f"[mimic-name] follower={_base_name(j)} leader={joints_dict[leader_key].get('output_name', _base_name(leader_key))} mult={mult} offset={offset}")
                else:
                    log(f"[skip-name] follower={j}: unsupported type {joints_dict[j]['type']}")
            return processed, unmatched

        _tick('Resolving mimics from joint names...')
        try:
            processed_mimics, unmatched_mimics = _annotate_mimics_from_names(joints_dict)
        except Exception:
            processed_mimics, unmatched_mimics = 0, 0
        if _check_cancel():
            if dlg: dlg.hide()
            ui.messageBox('Fusion2URDF was canceled', title)
            return 0
        
        # --------------------
        # Generate URDF (will include <mimic> for any linked joints)
        _tick('Writing URDF and launch files...')
        try:
            Write.write_urdf(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir)
            Write.write_materials_xacro(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir)
            Write.write_transmissions_xacro(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir)
            Write.write_gazebo_xacro(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir)
            Write.write_display_launch(package_name, robot_name, save_dir)
            Write.write_gazebo_launch(package_name, robot_name, save_dir)
            Write.write_control_launch(package_name, robot_name, save_dir, joints_dict)
            Write.write_yaml(package_name, robot_name, save_dir, joints_dict)
        except Exception:
            if dlg: dlg.hide()
            ui.messageBox('Failed while writing URDF/xacro/launch files:\n{}'.format(traceback.format_exc()), title)
            return 0
        if _check_cancel():
            if dlg: dlg.hide()
            ui.messageBox('Fusion2URDF was canceled', title)
            return 0
        
        # copy over package files
        utils.copy_package(save_dir, package_dir)
        utils.update_cmakelists(save_dir, package_name)
        utils.update_package_xml(save_dir, package_name)

        _tick('Exporting STL meshes...')
        # Generate STl files
        # copy_occs returns metadata about temporary components it created so we
        # can clean them up afterward and restore original names.
        try:
            copied_info = utils.copy_occs(root)
            utils.export_stl(design, save_dir, components)
        except Exception:
            # Still attempt cleanup below, but report export error
            if dlg: dlg.hide()
            ui.messageBox('Failed while exporting STL meshes:\n{}'.format(traceback.format_exc()), title)
            return 0
        # delete temporary copied components and restore original names
        try:
            utils.delete_copied_components(root, copied_info)
        except Exception:
            # best-effort cleanup; ignore errors here to avoid blocking the user
            pass
        
        try:
            if dlg: dlg.hide()
        except Exception:
            pass

        # Append joint summary to detailed log
        try:
            log('[summary] joints:')
            for jname, jd in joints_dict.items():
                base = f"[joint] name={jname} type={jd.get('type')} parent={jd.get('parent')} child={jd.get('child')}"
                if jd.get('mimic'):
                    mm = jd.get('mimic', {})
                    log(base + f" mimic={{joint:{mm.get('joint')}, multiplier:{mm.get('multiplier')}, offset:{mm.get('offset')}}}")
                else:
                    log(base + " mimic=None")
        except Exception:
            pass

        # Write detailed log to file
        try:
            log_path = os.path.join(save_dir, 'urdf_export_log.txt')
            with open(log_path, 'w', encoding='utf-8') as lf:
                lf.write('\n'.join(logs))
        except Exception:
            log_path = save_dir

        # Final success summary
        try:
            num_joints = len(joints_dict)
            num_mimic = sum(1 for j in joints_dict.values() if j.get('mimic'))
        except Exception:
            num_joints = 0
            num_mimic = 0
        ui.messageBox('URDF export complete.\n\nRobot: {}\nPackage: {}\nFolder: {}\nJoints: {}\nMimic followers annotated: {}\nName-based mimic unmatched: {}\nLog: {}'\
                      .format(robot_name, package_name, save_dir, num_joints, num_mimic, unmatched_mimics, log_path), title)
        
    except:
        try:
            _dlg = locals().get('dlg', None)
            if _dlg:
                _dlg.hide()
        except Exception:
            pass
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
