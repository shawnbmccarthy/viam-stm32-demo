import serial

from typing import Any, ClassVar, Mapping, Optional
from typing_extensions import Self

from viam.components.sensor.client import SensorClient
from viam.components.sensor.sensor import Sensor
from viam.components.sensor.service import SensorService
from viam.module.types import Reconfigurable
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName
from viam.resource.base import ResourceBase
from viam.resource.registry import ResourceRegistration, Registry
from viam.resource.types import Model, ModelFamily

from sensor_pb2 import SensorRequest, SensorResponse, SensorType
from wrapper_pb2 import WrappedRequest, WrappedResponse


class TimerOne(Sensor, Reconfigurable):
    MODEL: ClassVar[Model] = Model(
                                ModelFamily('demo', 'sensor'), 
                                'timer_one'
                             )
    tty: str

    @classmethod
    def new(
            cls,
            config: ComponentConfig,
            dependencies: Mapping[ResourceName, ResourceBase]
    ) -> Self:
        timer = cls(config.name)
        timer.tty = config.attributes.fields['tty'].string_value
        return timer

    async def get_readings(
        self,
        *,
        extra: Optional[Mapping[str, Any]] = None,
        timeout: Optional[float] = None, **kwargs
    ) -> Mapping[str, Any]:
        ret = {}
        wrapped_request = WrappedRequest()
        wrapped_response = WrappedResponse()
        wrapped_request.sensorRequest.sensor = SensorType.Counter
        with serial.Serial(port=self.tty, baudrate=115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE) as ser:
            x = ser.write(wrapped_request.ByteSize().to_bytes(1, 'little')) 
            ser.flush()
            y = ser.write(wrapped_request.SerializeToString())
            ser.flush()
            sz = ser.read(1)
            sz = int.from_bytes(sz, 'little')
            from_stm = ser.read(sz)
            wrapped_response = wrapped_response.FromString(from_stm)
            if not wrapped_response.status:
                ret['counter'] = 'error'
            ret['counter'] = wrapped_response.sensorResponse.value

        return ret

    def reconfigure(
        self, 
        config: ComponentConfig,
        dependencies: Mapping[ResourceName, ResourceBase]
    ):
        self.tty = config.attributes.fields['tty'].string_value


class TimerTwo(Sensor, Reconfigurable):
    MODEL: ClassVar[Model] = Model(
                                ModelFamily('demo', 'sensor'), 
                                'timer_two'
                             )
    tty: str

    @classmethod
    def new(
            cls,
            config: ComponentConfig,
            dependencies: Mapping[ResourceName, ResourceBase]
    ) -> Self:
        timer = cls(config.name)
        timer.tty = config.attributes.fields['tty'].string_value
        return timer

    async def get_readings(
        self,
        *,
        extra: Optional[Mapping[str, Any]] = None,
        timeout: Optional[float] = None, **kwargs
    ) -> Mapping[str, Any]:
        ret = {}
        temp_request = WrappedRequest()
        battery_request = WrappedRequest()
        temp_request.sensorRequest.sensor = SensorType.Temperature
        battery_request.sensorRequest.sensor = SensorType.Battery
        with serial.Serial(port=self.tty, baudrate=115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE) as ser:
            ret['temperature'] = self.do_reading(ser, temp_request)
            ret['battery'] = self.do_reading(ser, battery_request)
        return ret

    def do_reading(self, channel, req):
        response = WrappedResponse()
        w = channel.write(req.ByteSize().to_bytes(1, 'little'))
        channel.flush()
        w = channel.write(req.SerializeToString())
        channel.flush()
        sz = channel.read(1)
        sz = int.from_bytes(sz, 'little')
        from_stm = channel.read(sz)
        response = response.FromString(from_stm)
        if not response.status:
            return 'error'
        return response.sensorResponse.value


    def reconfigure(
        self, 
        config: ComponentConfig,
        dependencies: Mapping[ResourceName, ResourceBase]
    ):
        self.tty = config.attributes.fields['tty'].string_value

Registry.register_resource_creator(
    Sensor.SUBTYPE,
    TimerOne.MODEL,
    TimerOne.new
)

Registry.register_resource_creator(
    Sensor.SUBTYPE,
    TimerTwo.MODEL,
    TimerTwo.new
)
