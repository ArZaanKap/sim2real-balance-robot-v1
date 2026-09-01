# freeshaft.py — the BAM pieces we swap in for an open-loop, free-spinning wheel.
#
# WHY THIS FILE EXISTS
# --------------------
# BAM is built for smart SERVOS on a PENDULUM test bench: it drives a goal-position
# trajectory through the servo's internal P controller and identifies friction by how
# the actuator holds a known gravity load (mass * g * length * sin(angle)). We have
# none of that — a brushed GA25-370 + DRV8871, driven OPEN-LOOP at a commanded voltage,
# spinning a bare shaft (no gravity load). So we keep BAM's friction MATH and its
# MuJoCo/Warp export, and replace exactly two small things:
#
#   1) the testbench   -> FreeShaft: no gravity bias, all inertia in the motor armature.
#   2) the actuator    -> GA25Actuator: BAM's VoltageControlledActuator (which already
#                         models tau = kt*V/R - kt^2*w/R) subclassed only to add an
#                         `armature` inertia parameter + get_extra_inertia(), exactly the
#                         way MXActuator does. The firmware P-controller (compute_control,
#                         kp, error_gain) is never used because we replay recorded voltage
#                         with simulate_control=False.
#
# Everything else — models["m3"](), compute_frictions, the Simulator loop, to_mujoco —
# is stock BAM.

from bam.testbench import Testbench
from bam.actuator import VoltageControlledActuator
from bam.parameter import Parameter


class FreeShaft(Testbench):
    """Bare free-spinning shaft: no external (gravity) torque, constant inertia.

    BAM's Pendulum puts the load inertia here (mass*length^2) and a sin() gravity
    bias. For a wheel on the bench there is no gravity load, and the only inertia
    is the motor/gearbox armature — which GA25Actuator already contributes through
    get_extra_inertia(). So this testbench adds nothing: bias = 0, load inertia = 0.

    :param log: Log dict (unused; accepted so it drops into Actuator.load_log).
    """

    def __init__(self, log: dict):
        # Nothing to read — kept for API compatibility with Pendulum(log).
        pass

    def compute_mass(self, q: float, dq: float) -> float:
        # Load inertia is zero; the armature (get_extra_inertia) carries it all.
        return 0.0

    def compute_bias(self, q: float, dq: float) -> float:
        # No gravity load on a horizontal free shaft.
        return 0.0


class GA25Actuator(VoltageControlledActuator):
    """GA25-370 brushed gearmotor driven by a DRV8871 at a commanded voltage.

    This is stock VoltageControlledActuator (tau = kt*V/R - kt^2*w/R, control in
    volts) with one addition: an `armature` inertia parameter, so the free shaft
    has somewhere to store its rotational inertia. The control-law fields (kp,
    error_gain, max_pwm) exist only because the base class wants them; we never
    call compute_control (we replay recorded voltage, simulate_control=False), so
    their values are irrelevant to the fit.

    Electrical params kt and R are meant to be PINNED by hand from the datasheet
    STALL point (Plan A: R = V_stall / I_stall, kt = stall_torque / I_stall) and
    only the friction terms fitted — NOT from the no-load speed, which understates
    both (see the bam-ga25-identification-plan memory). Motor #1 came out kt=0.24,
    R=6.65. They are left as Parameters so you can free them if you add a current
    sensor.
    """

    def __init__(self, testbench_class, vin: float = 12.0):
        super().__init__(
            testbench_class,
            vin=vin,
            kp=0.0,          # unused: we never run the firmware P-controller
            error_gain=1.0,  # unused
            max_pwm=1.0,     # unused
        )

    def initialize(self):
        # Torque constant [Nm/A] (== back-EMF constant [V/(rad/s)]).
        # Pin from datasheet STALL: kt = stall_torque / stall_current (NOT no-load
        # speed — that understates it). Motor #1: 0.24. (seed value only; pinned in
        # the fit via --kt.)
        self.model.kt = Parameter(0.24, 0.0, 1.0)

        # Terminal resistance [Ohm]. Pin from stall: R = V_stall / I_stall. Motor #1: 6.65.
        self.model.R = Parameter(6.65, 0.05, 50.0)

        # Motor+gearbox apparent inertia reflected at the output shaft [kg m^2].
        # Fittable from the spin-up transient; small for a GA25 (wheel-on inflates it).
        self.model.armature = Parameter(1e-4, 1e-6, 3e-2)

    def get_extra_inertia(self) -> float:
        return self.model.armature.value
