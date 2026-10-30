#include "physics/TireModel.h"

#include <cmath>

namespace vsim {
namespace {

/// Sign function that returns +1 for zero, matching the `sgn alpha` of
/// [S3] Eqs. (14.13)/(14.14). At alpha = 0 the term it multiplies is itself
/// zero, so the choice does not affect the result.
double sgn(double v) { return (v < 0.0) ? -1.0 : 1.0; }

} // namespace

//===========================================================================
//  Construction - locate the peak of the universal curve.
//===========================================================================

TireModel::TireModel(const TireParameters& params) : m_p(params) {
    //-----------------------------------------------------------------------
    //  The universal curve (KB §2.20.2, [S3] Eqs. 14.5-14.7) is
    //
    //      F_bar = D' sin(theta),  theta = C' arctan(B' phi),
    //      phi   = (1 - E') x + (E'/B') arctan(B' x)
    //
    //  It peaks where the sine argument reaches pi/2, i.e. where
    //
    //      C' arctan(B' phi) = pi/2   ->   phi_peak = tan(pi/(2C')) / B'
    //
    //  phi(x) is monotonic, so the peak location in x follows by inversion.
    //  Newton converges in a handful of steps; phi'(x) is exact.
    //
    //  WHY THIS NUMBER MATTERS. Everything this project used to have to author
    //  about where a tyre saturates is recovered from it: the slip angle at
    //  peak lateral force (Eq. 14.4 inverted) and the longitudinal slip
    //  stiffness (Eq. 14.10 inverted, against the KB's quoted peak slip
    //  ratio). With [S3]'s own coefficients it comes out at about 2.68.
    //-----------------------------------------------------------------------
    const double B = m_p.curveB;
    const double C = m_p.curveC;
    const double E = m_p.curveE;

    if (!(B > 1e-9) || !(C > 1e-9)) return;

    const double phiPeak = std::tan(kPi / (2.0 * C)) / B;
    if (!isFinite(phiPeak) || !(phiPeak > 0.0)) return;

    double x = phiPeak;                       // phi(x) ~ x, a good first guess
    for (int i = 0; i < 60; ++i) {
        const double Bx  = B * x;
        const double f   = (1.0 - E) * x + (E / B) * std::atan(Bx) - phiPeak;
        const double df  = (1.0 - E) + E / (1.0 + Bx * Bx);
        if (!(std::abs(df) > 1e-12)) break;
        const double step = f / df;
        x -= step;
        if (std::abs(step) < 1e-12) break;
    }

    if (isFinite(x) && x > 1e-6) m_peakNormalisedSlip = x;
}

//===========================================================================
//  The universal curve
//===========================================================================

double TireModel::universalCurve(double x) const {
    //  Eqs. (14.5)-(14.7). Evaluated on |x| and re-signed, so the curve is odd:
    //  [S3] §11.2 records that the data are averaged between positive and
    //  negative slip angles before normalising, "which eliminates the values of
    //  lateral force at zero slip angle caused by tire asymmetries called
    //  conicity and ply steer" - i.e. the normalised tyre is symmetric by
    //  construction.
    const double a = std::abs(x);
    const double B = m_p.curveB;
    const double C = m_p.curveC;
    const double D = m_p.curveD;
    const double E = m_p.curveE;

    const double Ba  = B * a;
    const double phi = (1.0 - E) * a + (E / B) * std::atan(Ba);
    const double y   = D * std::sin(C * std::atan(B * phi));

    return (x < 0.0) ? -y : y;
}

//===========================================================================
//  Geometry - KB §2.15
//===========================================================================

double TireModel::deflection(double verticalLoad) const {
    if (!(m_p.verticalRate > 1e-6)) return 0.0;
    const double load = (verticalLoad > 0.0) ? verticalLoad : 0.0;
    return load / m_p.verticalRate;
}

double TireModel::loadedRadius(double verticalLoad, double unloadedRadius) const {
    // R_l = R_u - delta_z. Floored well above zero: a fully collapsed tire is
    // outside every model in the Knowledge Base.
    const double r = unloadedRadius - deflection(verticalLoad);
    return (r > 0.05 * unloadedRadius) ? r : 0.05 * unloadedRadius;
}

double TireModel::effectiveRollingRadius(double verticalLoad,
                                         double unloadedRadius) const {
    //     R_e = R_l + 2*delta_z/3   ==   R_u - delta_z/3           [Eq. 5.11]
    //
    // KB §2.15: the effective rolling radius sits two thirds of the way from
    // the loaded radius back up toward the unloaded radius, i.e. it falls by
    // about one third of the vertical deflection. Its consequence is worth
    // stating: R_e SHRINKS as load rises, so load transfer changes a wheel's
    // slip ratio even at constant speed and constant wheel speed.
    const double dz = deflection(verticalLoad);
    const double r  = unloadedRadius - dz / 3.0;
    return (r > 0.05 * unloadedRadius) ? r : 0.05 * unloadedRadius;
}

//===========================================================================
//  Load-dependent properties
//===========================================================================

double TireModel::frictionAt(double verticalLoad, double ratedLoad) const {
    //  KB §2.4.3 [S1]: "The friction coefficient DECREASES with increasing
    //  vertical load ... mu decreases by approximately 0.01 for each 10 %
    //  increase in load near the tire's rated load."
    //
    //      mu(F_z) = mu_y - k * (F_z/F_z0 - 1)          with k = 0.01/0.10
    //
    //  [S3] §6 names this "tire load sensitivity" and says it "has a major
    //  effect on the balance (plow/spin) of a race car operating near the
    //  limit", but supplies only plotted curves and no law - so [S1]'s rule is
    //  what is implemented. [S3]'s own Eagle GT-S numbers (1.10 at 900 lb
    //  falling to 0.96 at 1800 lb) are close to it and serve as a check.
    //
    //  This is the mechanism by which weight transfer reduces the TOTAL
    //  capability of an axle: the outside tire's gain is smaller than the
    //  inside tire's loss, because the curve bends downward.
    if (!(ratedLoad > 1e-6)) return m_p.lateralFriction;

    const double relative = verticalLoad / ratedLoad - 1.0;
    const double mu = m_p.lateralFriction - m_p.frictionLoadSensitivity * relative;

    // A tire cannot have negative friction. The KB gives the rule only "near
    // the tire's rated load" and says nothing about extrapolating it far, so
    // the floor is a guard against leaving the stated region of validity, not
    // a model of anything.
    return (mu > 0.01) ? mu : 0.01;
}

double TireModel::corneringStiffnessAt(double verticalLoad,
                                       double ratedLoad) const {
    //  KB Eq. 6-35/6-36, written per tire:
    //
    //      C_alpha = a*F_z - b*F_z^2
    //
    //  `a` is not authored. It is derived so that the cornering COEFFICIENT
    //  (KB Eq. 6-6, CC_alpha = C_alpha/F_z) has its authored value at the
    //  rated load:
    //
    //      CC_alpha = a - b*F_z0     =>     a = CC_alpha + b*F_z0
    //
    //  So authoring b changes the CURVATURE of C_alpha vs load without moving
    //  the value at the rated load - which is exactly the decomposition KB
    //  §2.6.4 relies on when it observes that in Eq. 6-41 "the linear term
    //  cancels: what one tire gains, the other loses. Only the curvature
    //  survives."
    const double a = m_p.corneringCoefficient + m_p.loadSensitivityB * ratedLoad;
    const double c = a * verticalLoad - m_p.loadSensitivityB * verticalLoad * verticalLoad;
    return (c > 0.0) ? c : 0.0;
}

double TireModel::camberStiffnessAt(double verticalLoad, double ratedLoad) const {
    //  KB §12.2 [S3]: camber stiffness is a fixed fraction of cornering
    //  stiffness - radial about 1/10, bias-ply 1/5 to 1/6.
    return m_p.camberStiffnessRatio * corneringStiffnessAt(verticalLoad, ratedLoad);
}

double TireModel::slipStiffnessAt(double verticalLoad, double ratedLoad) const {
    //  See the header for the derivation. In short:
    //      S_bar = k_x S /(mu_x F_z)         [Eq. (14.10)]
    //  and the universal curve peaks at S_bar = m_peakNormalisedSlip, so
    //  requiring the peak to land at the KB's quoted peak slip ratio gives k_x.
    if (!(m_p.peakSlipRatio > 1e-6)) return 0.0;

    const double muX = frictionAt(verticalLoad, ratedLoad) * m_p.longitudinalFrictionRatio;
    return m_peakNormalisedSlip * muX * verticalLoad / m_p.peakSlipRatio;
}

double TireModel::peakSlipAngle(double verticalLoad, double ratedLoad) const {
    //  Eq. (14.4) inverted at the curve's peak.
    const double Ca = corneringStiffnessAt(verticalLoad, ratedLoad);
    if (!(Ca > 1e-9)) return 0.0;

    const double muY = frictionAt(verticalLoad, ratedLoad);
    return std::atan(m_peakNormalisedSlip * muY * verticalLoad / Ca);
}

//===========================================================================
//  The tire block
//===========================================================================

TireForces TireModel::evaluate(const TireInputs& in) const {
    TireForces out;

    const double Fz = (in.verticalLoad > 0.0) ? in.verticalLoad : 0.0;

    //-----------------------------------------------------------------------
    //  0. An unloaded tire generates nothing.
    //
    //  KB §3.5 notes lateral load transfer can reach inside-wheel lift, and
    //  §2.6.4 says the load-sensitivity linearisation "fails entirely" there.
    //  A lifted wheel producing zero force is the one thing that is certainly
    //  right.
    //-----------------------------------------------------------------------
    if (!(Fz > 1e-3)) {
        out.frictionAvailable = m_p.lateralFriction;
        return out;
    }

    //-----------------------------------------------------------------------
    //  1. The load-dependent inputs the nondimensional model consumes.
    //
    //  KB §2.20.4 [LIMIT] is explicit that mu_y, mu_x, C_alpha, k_x, C_gamma
    //  and t "are load-dependent INPUTS, not predictions" - the model does not
    //  produce them, it consumes them. Here they come from the [S1] load
    //  relations above and the [S3] stiffness ratios.
    //-----------------------------------------------------------------------
    const double muY = frictionAt(Fz, in.ratedLoad);
    const double muX = muY * m_p.longitudinalFrictionRatio;

    const double Ca = corneringStiffnessAt(Fz, in.ratedLoad);
    const double Cg = camberStiffnessAt(Fz, in.ratedLoad);
    const double kx = slipStiffnessAt(Fz, in.ratedLoad);

    out.frictionAvailable  = muY;
    out.frictionLimit      = muY * Fz;
    out.corneringStiffness = Ca;
    out.camberStiffness    = Cg;
    out.slipStiffness      = kx;

    //-----------------------------------------------------------------------
    //  2. NORMALISE  -  KB §2.20.2, Eqs. (14.4), (14.8), (14.10).
    //
    //      a_bar = C_alpha tan(alpha) /(mu_y F_z)
    //      g_bar = C_gamma sin(gamma) /(mu_y F_z)
    //      S_bar = k_x S /(mu_x F_z)
    //
    //  [S3] p.476: "F_bar, M_bar_z and M_bar_x are each functions of a_bar,
    //  INDEPENDENT OF THE LOAD, because load is already included in the above
    //  relations as well as in determining the friction coefficient, cornering
    //  stiffness, pneumatic trail and overturning trail."
    //-----------------------------------------------------------------------
    const double denomY = muY * Fz;
    const double denomX = muX * Fz;

    const double tanA = std::tan(in.slipAngle);
    const double aBar = (denomY > 1e-9) ? Ca * tanA / denomY : 0.0;
    const double gBar = (denomY > 1e-9) ? Cg * std::sin(in.camberAngle) / denomY : 0.0;
    const double sBar = (denomX > 1e-9) ? kx * in.slipRatio / denomX : 0.0;

    //-----------------------------------------------------------------------
    //  3. COMBINE SLIP WITH CAMBER  -  Eq. (14.13).
    //
    //      beta_bar = a_bar /(1 - g_bar * sgn(alpha))
    //
    //  [S3] p.480: "FN reduces to F_bar for zero camber angle. Now FN is a
    //  function of beta_bar, independent of load. When the slip angle is zero,
    //  beta_bar is zero, FN is zero and Eq. (14.14) reduces to F_bar = g_bar" -
    //  i.e. pure camber thrust, the straight line of [S3] Fig. 14.3.
    //
    //  [S3] states Eqs. (14.13)/(14.14) "for positive camber angles"; applying
    //  them to negative camber as well is an implementation interpretation, and
    //  the KB flags this step as such (KB §2.20.4). The denominator is guarded
    //  because g_bar -> 1 would make it singular; that bound corresponds to
    //  camber thrust alone reaching the friction limit, well outside the
    //  model's stated +/-10 deg camber validity.
    //-----------------------------------------------------------------------
    double camberScale = 1.0 - gBar * sgn(in.slipAngle);
    if (std::abs(camberScale) < 0.05) camberScale = std::copysign(0.05, camberScale);

    const double betaBar = aBar / camberScale;

    //-----------------------------------------------------------------------
    //  4. THE RESULTANT  -  Eqs. (14.15), (14.16).
    //
    //      k = sqrt(S_bar^2 + beta_bar^2)      R = R(k)
    //
    //  This is the whole model in two lines. [S3] p.44: "the tire only knows a
    //  resultant force and a resultant slip velocity." R(k) is the SAME
    //  universal curve, verified by [S3] over slip angles from 1 to 28 deg.
    //-----------------------------------------------------------------------
    const double k = std::sqrt(sBar * sBar + betaBar * betaBar);
    const double R = universalCurve(k);

    out.normalisedSlip = k;
    out.frictionUsage  = std::abs(R);

    //-----------------------------------------------------------------------
    //  5. SPLIT THE RESULTANT  -  Eqs. (14.18)-(14.21).
    //
    //      eta_0 = C_alpha mu_x /(k_x mu_y)
    //      eta(k) = (1+eta_0)/2 - (1-eta_0)/2 cos(k/2)   for |k| <= 2pi
    //             = 1                                    for |k| >  2pi
    //      F_bar   = eta R tan(alpha) / sqrt(S^2 + eta^2 tan^2(alpha))
    //      F_bar_x =     R     S      / sqrt(S^2 + eta^2 tan^2(alpha))
    //
    //  [S3] p.483 on why eta exists: it "is necessary to make Eq. (14.17) hold
    //  for both small and large slip angles and slip ratios", and it is exactly
    //  the factor that makes the combined model degenerate to the two pure-slip
    //  linear stiffnesses at small slip - F_y = C_alpha tan(alpha) and
    //  F_x = k_x S simultaneously.
    //
    //  NOTE WHAT IS NOT HERE: no friction circle is applied. Squaring and
    //  adding (14.20) and (14.21) gives F_bar^2 + F_bar_x^2 = R^2 identically,
    //  and R <= D' = 1, so the friction ellipse holds as an identity rather
    //  than as a clamp. That is the whole difference between having a
    //  combined-slip model and having only a capability boundary.
    //-----------------------------------------------------------------------
    const double eta0 = (kx * muY > 1e-9) ? (Ca * muX) / (kx * muY) : 1.0;
    const double eta  = (std::abs(k) <= kTwoPi)
                            ? 0.5 * (1.0 + eta0) - 0.5 * (1.0 - eta0) * std::cos(k / 2.0)
                            : 1.0;

    const double S = in.slipRatio;
    const double etaTan = eta * tanA;
    const double split = std::sqrt(S * S + etaTan * etaTan);

    double FN   = 0.0;   // normalised lateral force, still camber-combined
    double FxBar = 0.0;

    if (split > 1e-12) {
        FN    = eta * R * tanA / split;
        FxBar = R * S / split;
    } else {
        // Neither slipping nor spinning: the only force left is camber thrust,
        // which step 6 restores. [S3] p.480 gives exactly this reduction.
        FN    = 0.0;
        FxBar = 0.0;
    }

    //-----------------------------------------------------------------------
    //  6. RESTORE CAMBER  -  inverse of Eq. (14.14).
    //
    //      F_bar = FN (1 - g_bar sgn(alpha)) + g_bar
    //
    //  At zero slip angle this leaves F_bar = g_bar, i.e. pure camber thrust
    //  F_y = C_gamma sin(gamma) - "a lateral force in the direction of the
    //  tilt" [S3] p.46. That is the camber thrust of KB §2.7, arriving here
    //  as a consequence of the normalisation rather than as a separate term.
    //-----------------------------------------------------------------------
    const double FBar = FN * camberScale + gBar;

    //-----------------------------------------------------------------------
    //  7. DE-NORMALISE  -  Eqs. (14.1), (14.9).
    //-----------------------------------------------------------------------
    double Fy = FBar * denomY;
    double Fx = FxBar * denomX;

    //-----------------------------------------------------------------------
    //  8. RELAXATION LENGTH - the one tire transient any source models.
    //
    //  KB §2.11 [S1]: lateral force does not appear instantaneously; the tread
    //  band must be distorted, which takes about half to one tire revolution of
    //  ROLLING DISTANCE. KB §2.19 [S2] adds the functional form: a first-order
    //  lag with time constant tau = (relaxation length)/V, so it shrinks as
    //  speed rises, and states the consequence of leaving it out bluntly - the
    //  tire would hand the vehicle its full steady-state force in the same
    //  timestep the slip angle appears, which is physically impossible.
    //
    //  [S3] has no tyre transient content at all (KB §12.2), so this stays
    //  [S1]/[S2] material and is applied to F_y only. KB §2.11 is explicit that
    //  lateral relaxation is "the ONLY transient tire effect the book models";
    //  longitudinal force transients are gap G6, still open in all three
    //  sources.
    //
    //  Integrated in exponential form rather than as F += (Fss-F)*dt/tau. The
    //  two agree to first order; the exponential cannot overshoot when dt
    //  approaches tau, which happens at high speed where tau is small.
    //-----------------------------------------------------------------------
    const double speed = std::abs(in.rollingSpeed);
    if (in.dt > 0.0 && speed > 1e-6 && in.unloadedRadius > 1e-6) {
        const double relaxLength =
            m_p.relaxationLengthRevolutions * kTwoPi *
            effectiveRollingRadius(Fz, in.unloadedRadius);

        if (relaxLength > 1e-6) {
            const double tau   = relaxLength / speed;
            const double alpha = 1.0 - std::exp(-in.dt / tau);
            Fy = in.previousLateralForce + (Fy - in.previousLateralForce) * alpha;
        }
    }

    //-----------------------------------------------------------------------
    //  9. ALIGNING MOMENT - KB §2.8.
    //
    //      M_z = -F_y * t
    //
    //  The pneumatic trail t is the offset by which the resultant lateral force
    //  acts BEHIND the centre of the contact patch, because lateral shear
    //  builds progressively from the front of the patch. [S3] Eq. (14.2) calls
    //  the same quantity T_z, "the ratio of self-aligning torque to lateral
    //  force for small slip angles".
    //
    //  t is not authored. It is recovered from the KB's two independently
    //  quoted tire coefficients - the aligning moment coefficient (M_z per unit
    //  load per degree, KB §2.8) and the cornering stiffness - since at small
    //  slip M_z = coeff*F_z*alpha and F_y = C_alpha*alpha, so
    //
    //      t = coeff * F_z / C_alpha
    //
    //  LIMITATION, stated because it is a real loss of behaviour. KB §2.8
    //  describes t decreasing, reaching zero and REVERSING as the sliding
    //  region grows, so that M_z peaks and decays BEFORE F_y saturates - which
    //  is the steering-feel cue that warns a driver of the limit. [S3] adds
    //  numbers to that description (KB §12.2: M_z peaks at about HALF the slip
    //  angle of the F_y peak, and the rim force falls about 30 % in the last
    //  degree before breakaway) but still no t(alpha, F_z) function. Gap G4 is
    //  PARTIAL in all three sources, so a constant trail it remains - the same
    //  form KB Eq. 6-61 and Eq. 8-8 use.
    //-----------------------------------------------------------------------
    const double trail = (Ca > 1e-9) ? m_p.aligningMomentCoefficient * Fz / Ca : 0.0;
    out.pneumaticTrail = trail;
    out.aligningMoment = -Fy * trail;

    //-----------------------------------------------------------------------
    // 10. Local slope of the longitudinal curve, for the caller's integrator.
    //     See the note on TireForces::longitudinalSlipSlope.
    //-----------------------------------------------------------------------
    {
        //  Taken on the PURE longitudinal curve, F_x = R(S_bar) * mu_x F_z with
        //  S_bar = k_x S/(mu_x F_z), so dF_x/dS = R'(S_bar) * k_x. Under
        //  combined slip the true slope is smaller than this, so using the pure
        //  one over-damps the caller's integrator rather than under-damping it,
        //  which is the safe direction. Like the pure curve it goes to zero at
        //  the peak and negative beyond, and is floored at zero so the
        //  integrator never anti-damps.
        constexpr double kh = 1.0e-3;
        const double sBarAbs = std::abs(sBar);
        const double dR = (universalCurve(sBarAbs + kh) - universalCurve(sBarAbs - kh))
                          / (2.0 * kh);
        const double slope = dR * kx;
        out.longitudinalSlipSlope = (isFinite(slope) && slope > 0.0) ? slope : 0.0;
    }

    out.longitudinal = Fx;
    out.lateral      = Fy;
    return out;
}

} // namespace vsim
