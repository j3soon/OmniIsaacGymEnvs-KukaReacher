# Copyright (c) 2018-2022, NVIDIA Corporation
# Copyright (c) 2022-2023, Johnson Sun
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Ref: https://docs.omniverse.nvidia.com/isaacsim/latest/advanced_tutorials/tutorial_advanced_import_urdf.html#importing-urdf-using-python

import os

import omni.kit.commands
import omni.usd
from omni.importer.urdf import _urdf
from omni.isaac.core.utils.extensions import get_extension_path_from_name
from pxr import Sdf, UsdGeom


def create_kuka_from_urdf(urdf_path, usd_path, mesh_usd_path, instanceable_usd_path):
    # Set the settings in the import config
    import_config = _urdf.ImportConfig()
    import_config.merge_fixed_joints = False
    import_config.convex_decomp = False
    import_config.import_inertia_tensor = False
    import_config.fix_base = True
    import_config.make_default_prim = True
    import_config.self_collision = False
    import_config.create_physics_scene = True
    # The two values below follows the suggestion from Step 2 in
    # `5.1.3. Using the URDF Importer Extension Window`.
    # Ref: https://docs.omniverse.nvidia.com/isaacsim/latest/advanced_tutorials/tutorial_advanced_import_urdf.html#using-the-urdf-importer-extension-window
    # However, we found that the drive strength is too large, so we reduced it to the same value as drive damping.
    import_config.default_drive_strength = 100000.0 # 10000000.0
    import_config.default_position_drive_damping = 100000.0
    import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
    import_config.distance_scale = 1
    import_config.density = 0.0
    # Finally import the robot & save it as USD
    result, prim_path = omni.kit.commands.execute(
        "URDFParseAndImportFile", urdf_path=urdf_path,
        import_config=import_config, dest_path=usd_path,
    )
    import_config.make_instanceable=True
    import_config.instanceable_usd_path=mesh_usd_path
    # Finally import the robot & save it as instanceable USD
    result, prim_path = omni.kit.commands.execute(
        "URDFParseAndImportFile", urdf_path=urdf_path,
        import_config=import_config, dest_path=instanceable_usd_path,
    )

if __name__ == '__main__':
    kuka_urdf_path = f'{os.path.expanduser("~")}/OmniIsaacGymEnvs-KukaReacher/thirdparty/kuka_kr120_support/urdf/kr120r2500pro.urdf'
    kuka_usd_path = 'omniverse://localhost/Projects/J3soon/Isaac/2023.1.0/Isaac/Robots/Kuka/KR120_R2500_Pro/kr120r2500pro_urdf.usd'
    kuka_mesh_usd_path = 'omniverse://localhost/Projects/J3soon/Isaac/2023.1.0/Isaac/Robots/Kuka/KR120_R2500_Pro/kr120r2500pro_urdf_instanceable_meshes.usd'
    kuka_instanceable_usd_path = 'omniverse://localhost/Projects/J3soon/Isaac/2023.1.0/Isaac/Robots/Kuka/KR120_R2500_Pro/kr120r2500pro_urdf_instanceable.usd'
    create_kuka_from_urdf(kuka_urdf_path, kuka_usd_path, kuka_mesh_usd_path, kuka_instanceable_usd_path)
    print("Done!")
