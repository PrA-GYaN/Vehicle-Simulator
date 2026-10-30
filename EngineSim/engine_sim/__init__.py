from .config import EngineConfig, load_config
from .state import EngineState
from .engine import Engine
from .telemetry import TelemetryRecorder, TelemetryEntry

__all__ = [
    "EngineConfig",
    "load_config",
    "EngineState",
    "Engine",
    "TelemetryRecorder",
    "TelemetryEntry",
]

# v0.3 new modules (importable but not required for basic usage)
from .gas_properties import cp_air, cv_air, gamma_air
from .manifold_model import manifold_pressure_dynamics, throttle_flow_rate
from .spark_model import mbt_timing, combustion_duration
from .heat_transfer import woschni_heat_transfer_coeff, annand_heat_transfer_coeff
from .fuel_film import FuelFilmState, fuel_film_dynamics
from .lambda_controller import LambdaController
from .cyclic_variability import cyclic_variability_cov
from .turbo_model import TurboState, compressor_power, turbine_power
from .emissions_model import nox_emission_index, hc_emissions_fraction
from .friction_breakdown import total_friction_breakdown
from .diesel_combustion import ignition_delay, watson_heat_release
