import serial

from typing import Any, ClassVar, Dict, Mapping, Optional, Tuple
from typing_extensions import Self

from viam.components.motor.motor import Motor
from viam.module.types import Reconfigurable
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName
from viam.resource.base import ResourceBase
from viam.resource.registry import ResourceRegistration, Registry
from viam.resource.types import Model, ModelFamily

from motor_pb2 import MotorRequest, MotorResponse, Action
from wrapper_pb2 import WrappedRequest, WrappedResponse


class StmMotor(Motor, Reconfigurable):
    MODEL: ClassVar[Model] = Model(
                                ModelFamily('demo', 'motor'), 
                                'stm_motor'
                             )
    tty: str

    @classmethod
    def new(
            cls,
            config: ComponentConfig,
            dependencies: Mapping[ResourceName, ResourceBase]
    ) -> Self:
        motor = cls(config.name)
        motor.tty = config.attributes.fields['tty'].string_value
        return motor

    def send_msg(self, req):
        wrapped_response = WrappedResponse()
        with serial.Serial(port=self.tty, baudrate=115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE) as ser:
            x = ser.write(req.ByteSize().to_bytes(1, 'little'))
            ser.flush()
            y = ser.write(req.SerializeToString())
            ser.flush()
            sz = ser.read(1)
            sz = int.from_bytes(sz, 'little')
            from_stm = ser.read(sz)
            wrapped_response = wrapped_response.FromString(from_stm)
        return wrapped_response
        
    async def set_power(self, power: float, *, extra: Optional[Dict[str, Any]] = None, timout: Optional[float] = None, **kwargs):
        wrapped_request = WrappedRequest()
        wrapped_request.motorRequest.action = Action.SetPower
        wrapped_request.motorRequest.power = power
        wrapped_response = self.send_msg(wrapped_request)
        if not wrapped_response.status:
            # failure
            pass

    async def go_for(self, rpm: float, revolutions: float, *, extra: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None, **kwargs):
        print(f'go_for -> {rpm}, {revolutions}')

    async def go_to(self, pm: float, position_revolutions: float, *, extra: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None, **kwargs):
        print(f'go_to -> {pm}, {position_revolutions}')

    async def reset_zero_position(self, offset: float, *, extra: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None, **kwargs):
        print(f'reset_zero_position -> {offset}')

    async def get_position(self, *, extra: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None, **kwargs) -> float:
        wrapped_request = WrappedRequest()
        wrapped_request.motorRequest.action = Action.GetPosition
        wrapped_response = self.send_msg(wrapped_request)
        if not wrapped_response.status:
            # failure
            return -1.0
        if not wrapped_response.motorResponse.HasField('val_f'):
            return -2.0
        return wrapped_response.motorResponse.val_f

    async def get_properties(self, *, extra: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None, **kwargs) -> Motor.Properties:
        return Motor.Properties(False)

    async def stop(self, *, extra: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None, **kwargs):
        await self.set_power(0.0)

    async def is_powered(self, *, extra: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None, **kwargs) -> Tuple[bool, float]:
        wrapped_request = WrappedRequest()
        wrapped_request.motorRequest.action = Action.IsPowered
        wrapped_response = self.send_msg(wrapped_request)
        is_powered = False
        current_power = 0.0
        if not wrapped_response.status:
            return (is_powered, current_power)
        if wrapped_response.motorResponse.HasField('val_f'):
            current_power = wrapped_response.motorResponse.val_f
        if wrapped_response.motorResponse.HasField('val_b'):
            is_powered = wrapped_response.motorResponse.val_b
        print(f'IS_POWERED ===========================> is_powered:{is_powered}, current_power:{current_power} ====================')
        return (is_powered, current_power)

    async def is_moving(self) -> bool:
        wrapped_request = WrappedRequest()
        wrapped_request.motorRequest.action = Action.IsMoving
        wrapped_response = self.send_msg(wrapped_request)
        if not wrapped_response.status:
            return False
        if not wrapped_response.motorResponse.HasField('val_b'):
            return False
        print(f'IS_MOVING ===========================> is_moving: {wrapped_response.motorResponse.val_b} ==================>')
        return wrapped_response.motorResponse.val_b

    def reconfigure(
        self, 
        config: ComponentConfig,
        dependencies: Mapping[ResourceName, ResourceBase]
    ):
        self.tty = config.attributes.fields['tty'].string_value


Registry.register_resource_creator(
    Motor.SUBTYPE,
    StmMotor.MODEL,
    StmMotor.new
)
