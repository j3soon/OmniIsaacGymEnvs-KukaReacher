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

# Ref: /omniisaacgymenvs/tasks/shadow_hand.py


import math

import numpy as np
import torch
# from omniisaacgymenvs.sim2real.kukakr120r2500pro import RealWorldKukaKR120R2500Pro
from omniisaacgymenvs.utils.config_utils.sim_config import SimConfig
from omniisaacgymenvs.robots.articulations.views.kukakr120r2500pro_view import KukaKR120R2500ProView
from omniisaacgymenvs.robots.articulations.kukakr120r2500pro import KukaKR120R2500Pro
from omniisaacgymenvs.tasks.shared.reacher import ReacherTask

from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.utils.torch import *
from omni.isaac.gym.vec_env import VecEnvBase


class KukaKR120R2500ProReacherTask(ReacherTask):
    def __init__(
        self,
        name: str,
        sim_config: SimConfig,
        env: VecEnvBase,
        offset=None
    ) -> None:
        self.update_config(sim_config)
        ReacherTask.__init__(self, name=name, env=env)
        return

    def update_config(self, sim_config):
        self._sim_config = sim_config
        self._cfg = sim_config.config
        self._task_cfg = sim_config.task_config

        self.obs_type = self._task_cfg["env"]["observationType"]
        if not (self.obs_type in ["full"]):
            raise Exception(
                "Unknown type of observations!\nobservationType should be one of: [full]")
        print("Obs type:", self.obs_type)
        self.num_obs_dict = {
            "full": 29,
            # 6: Kuka joints position (action space)
            # 6: Kuka joints velocity
            # 3: goal position
            # 4: goal rotation
            # 4: goal relative rotation
            # 6: previous action
        }

        self.object_scale = torch.tensor([1.0] * 3)
        self.goal_scale = torch.tensor([2.0] * 3)

        self._num_observations = self.num_obs_dict[self.obs_type]
        self._num_actions = 6
        self._num_states = 0

        pi = math.pi
        # For actions
        # The dof limits follow those defined in:
        # thirdparty/kuka_kr120_support/urdf/kr120r2500pro.urdf
        self._dof_limits = torch.tensor([[
            [np.deg2rad(-185), np.deg2rad(185)],
            [np.deg2rad(-155), np.deg2rad(35)],
            [np.deg2rad(-130), np.deg2rad(154)],
            [np.deg2rad(-350), np.deg2rad(350)],
            [np.deg2rad(-130), np.deg2rad(130)],
            [np.deg2rad(-350), np.deg2rad(350)],
        ]], dtype=torch.float32, device=self._cfg["sim_device"])
        self.useURDF = self._task_cfg["env"]["useURDF"]

        # Setup Sim2Real
        sim2real_config = self._task_cfg['sim2real']
        if sim2real_config['enabled'] and self.test and self.num_envs == 1:
            raise NotImplementedError("Sim2Real is not implemented for Kuka KR120R2500Pro yet!")
        ReacherTask.update_config(self)

    def get_num_dof(self):
        return self._arms.num_dof

    def get_arm(self):
        if not self.useURDF:
            usd_path = "omniverse://localhost/Projects/J3soon/Isaac/2023.1.0/Isaac/Robots/Kuka/KR120_R2500_Pro/kr120r2500pro_urdf_instanceable.usd"
        else:
            raise NotImplementedError("Only URDF is supported for Kuka KR120R2500Pro now!")
        kuka = KukaKR120R2500Pro(
            prim_path=self.default_zero_env_path + "/Kuka",
            name="Kuka",
            usd_path=usd_path
        )
        self._sim_config.apply_articulation_settings(
            "kuka",
            get_prim_at_path(kuka.prim_path),
            self._sim_config.parse_actor_config("kuka"),
        )

    def get_arm_view(self, scene):
        end_effector_prim_paths_expr = "/World/envs/.*/Kuka/tool0"
        arm_view = KukaKR120R2500ProView(
            prim_paths_expr="/World/envs/.*/Kuka",
            end_effector_prim_paths_expr=end_effector_prim_paths_expr,
            name="kuka_view"
        )
        scene.add(arm_view._end_effectors)
        return arm_view

    def get_object_displacement_tensor(self):
        return torch.tensor([0.05, 0.0, 0.0], device=self.device).repeat((self.num_envs, 1))

    def get_observations(self):
        self.arm_dof_pos = self._arms.get_joint_positions()
        self.arm_dof_vel = self._arms.get_joint_velocities()

        if self.obs_type == "full_no_vel":
            self.compute_full_observations(True)
        elif self.obs_type == "full":
            self.compute_full_observations()
        else:
            print("Unkown observations type!")

        observations = {self._arms.name: {"obs_buf": self.obs_buf}}
        return observations

    def get_reset_target_new_pos(self, n_reset_envs):
        # Randomly generate goal positions, although the resulting goal may still not be reachable.
        new_pos = torch_rand_float(-1, 1, (n_reset_envs, 3), device=self.device)
        new_pos[:, 0] = new_pos[:, 0] * 0.5 + 1.0 * torch.sign(new_pos[:, 0])
        new_pos[:, 1] = new_pos[:, 1] * 0.5 + 1.0 * torch.sign(new_pos[:, 1])
        new_pos[:, 2] = torch.abs(new_pos[:, 2] * 1.3) + 0.2
        return new_pos

    def compute_full_observations(self, no_vel=False):
        if no_vel:
            raise NotImplementedError()
        else:
            # There are many redundant information for the simple Reacher task, but we'll keep them for now.
            self.obs_buf[:, 0:self.num_arm_dofs] = unscale(self.arm_dof_pos[:, :self.num_arm_dofs],
                self.arm_dof_lower_limits, self.arm_dof_upper_limits)
            self.obs_buf[:, self.num_arm_dofs:2*self.num_arm_dofs] = self.vel_obs_scale * self.arm_dof_vel[:, :self.num_arm_dofs]
            base = 2 * self.num_arm_dofs
            self.obs_buf[:, base+0:base+3] = self.goal_pos
            self.obs_buf[:, base+3:base+7] = self.goal_rot
            self.obs_buf[:, base+7:base+11] = quat_mul(self.object_rot, quat_conjugate(self.goal_rot))
            self.obs_buf[:, base+11:base+17] = self.actions

    def send_joint_pos(self, joint_pos):
        # self.real_world_kukakr120r2500pro.send_joint_pos(joint_pos)
        pass
