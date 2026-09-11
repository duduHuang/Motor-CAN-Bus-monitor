from .param_command import ParamCommandEncoder
from .read_command import ReadCommandEncoder
from .motion_command import MotionCommandEncoder
from .system_command import SystemCommandEncoder
from .mit_command import MITCommandEncoder, MITCommand, MITConfig

class MotorProtocol(
    ParamCommandEncoder,
    ReadCommandEncoder,
    MotionCommandEncoder,
    SystemCommandEncoder
):
    """統一對外發送指令門面 (包含伺服指令與 MIT 模式壓碼)"""

    @classmethod
    def mit_control(
        cls, 
        p_des: float, 
        v_des: float, 
        kp: float, 
        kd: float, 
        t_ff: float,
        cfg: MITConfig = MITConfig()
    ) -> bytes:
        """MIT 模式 8-byte 壓碼介面"""
        cmd = MITCommand(
            p_des = p_des,
            v_des = v_des,
            kp = kp,
            kd = kd,
            t_ff = t_ff
        )
        return MITCommandEncoder.encode(cmd, cfg)

# 保持相容性別名
SingleMotorProtocol = MotorProtocol