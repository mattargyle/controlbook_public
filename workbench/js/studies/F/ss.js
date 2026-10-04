// Study F, Chapters 11–14 (F.11–F.14): full state feedback on the decoupled
// longitudinal (h, ḣ; input F̃) and lateral (z, θ, ż, θ̇; input τ) models,
// integral augmentation, observers, and disturbance observers.
//
// Lateral observer: the outputs are z and θ, so L is 4×2 and not unique. The
// workbench uses the block structure L = [[L_z1, 0], [0, L_θ1], [L_z2, 0], [0, L_θ2]]
// (θ innovation feeds θ̂, z innovation feeds ẑ). Then A − LC is block triangular
// and eig(A − LC) = eig(z block) ∪ eig(θ block): two SISO designs.
//
// Work mode: the time plots run the student's own Python controller (F.11(e),
// F.12(a, c), F.13(c), F.14(b); WB.myCtrl). There are no gain sliders or
// implementation toggles; checks compare with the design where the problem fixes
// it and check behaviour (specs, tracking, estimates) where the student tunes.
(function () {
  const { el, slider, segmented, section, bind } = WB.ui;
  const M = WB.math;
  const L = WB.la;
  const F = WB.F;
  const T = WB.tf;
  const { tex, texMat, texPole, fmt, fmtPole } = M;
  const PD = () => WB.pd;
  const D = () => WB.design;

  // Solution controllers (the reference designs: F.8 pole pairs, p_I = −0.4,
  // observers 10× faster, disturbance poles −1, −1, −10).
  const SOL = {
    f11: String.raw`import control as cnt

class Controller:
    def __init__(self):
        M = P.mc + 2 * P.mr
        J = P.Jc + 2 * P.mr * P.d**2
        a = P.mu / M
        # F.6 models: x_lon = (h, hdot), x_lat = (z, theta, zdot, thetadot)
        A_lon = np.array([[0.0, 1.0], [0.0, 0.0]])
        B_lon = np.array([[0.0], [1 / M]])
        C_lon = np.array([[1.0, 0.0]])
        A_lat = np.array([[0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0], [0.0, -P.g, -a, 0.0], [0.0, 0.0, 0.0, 0.0]])
        B_lat = np.array([[0.0], [0.0], [0.0], [1 / J]])
        C_z = np.array([[1.0, 0.0, 0.0, 0.0]])
        # (a) poles: the F.8 pairs (t_r,h = 8 s, t_r,theta = 0.8 s, t_r,z = 8 s, zeta = 0.707)
        zeta = 0.707
        pair = lambda tr: np.roots([1, 2 * zeta * 2.2 / tr, (2.2 / tr)**2])
        self.K_lon = cnt.place(A_lon, B_lon, pair(8.0))
        self.K_lat = cnt.place(A_lat, B_lat, np.concatenate([pair(8.0), pair(0.8)]))
        # (d) reference gains for unity DC gain (Eq. 11.35)
        self.kr_lon = -1.0 / (C_lon @ np.linalg.inv(A_lon - B_lon @ self.K_lon) @ B_lon)[0, 0]
        self.kr_lat = -1.0 / (C_z @ np.linalg.inv(A_lat - B_lat @ self.K_lat) @ B_lat)[0, 0]
        self.Fe = M * P.g
        self.unmix = np.linalg.inv(np.array([[1.0, 1.0], [P.d, -P.d]]))

    def update(self, r, x):
        h_r, z_r = r[0, 0], r[1, 0]
        z, h, theta, zdot, hdot, thetadot = x[:, 0]
        x_lon = np.array([[h], [hdot]])
        x_lat = np.array([[z], [theta], [zdot], [thetadot]])
        F = self.Fe - (self.K_lon @ x_lon)[0, 0] + self.kr_lon * h_r
        tau = -(self.K_lat @ x_lat)[0, 0] + self.kr_lat * z_r
        return np.clip(self.unmix @ np.array([F, tau]), 0, P.f_max)
`,
    f12: String.raw`import control as cnt

class Controller:
    def __init__(self):
        M = P.mc + 2 * P.mr
        J = P.Jc + 2 * P.mr * P.d**2
        a = P.mu / M
        A_lon = np.array([[0.0, 1.0], [0.0, 0.0]])
        B_lon = np.array([[0.0], [1 / M]])
        C_lon = np.array([[1.0, 0.0]])
        A_lat = np.array([[0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0], [0.0, -P.g, -a, 0.0], [0.0, 0.0, 0.0, 0.0]])
        B_lat = np.array([[0.0], [0.0], [0.0], [1 / J]])
        C_z = np.array([[1.0, 0.0, 0.0, 0.0]])
        # augmented with x_I = integral of (r - y) (Eq. 12.1)
        aug = lambda A, B, C: (np.block([[A, np.zeros((A.shape[0], 1))], [-C, np.zeros((1, 1))]]), np.vstack([B, [[0.0]]]))
        A1_lon, B1_lon = aug(A_lon, B_lon, C_lon)
        A1_lat, B1_lat = aug(A_lat, B_lat, C_z)
        zeta = 0.707
        pair = lambda tr: np.roots([1, 2 * zeta * 2.2 / tr, (2.2 / tr)**2])
        p_I_h, p_I_z = -0.4, -0.4
        K1 = cnt.place(A1_lon, B1_lon, np.concatenate([pair(8.0), [p_I_h]]))
        self.K_lon, self.ki_lon = K1[:, 0:2], K1[0, 2]
        K1 = cnt.place(A1_lat, B1_lat, np.concatenate([pair(8.0), pair(0.8), [p_I_z]]))
        self.K_lat, self.ki_lat = K1[:, 0:4], K1[0, 4]
        self.Fe = M * P.g
        self.unmix = np.linalg.inv(np.array([[1.0, 1.0], [P.d, -P.d]]))
        self.int_h = 0.0
        self.int_z = 0.0
        self.e_h_prev = 0.0
        self.e_z_prev = 0.0

    def forces(self, x_lon, x_lat, int_h, int_z):
        F = self.Fe - (self.K_lon @ x_lon)[0, 0] - self.ki_lon * int_h
        tau = -(self.K_lat @ x_lat)[0, 0] - self.ki_lat * int_z
        return self.unmix @ np.array([F, tau])

    def update(self, r, x):
        h_r, z_r = r[0, 0], r[1, 0]
        z, h, theta, zdot, hdot, thetadot = x[:, 0]
        x_lon = np.array([[h], [hdot]])
        x_lat = np.array([[z], [theta], [zdot], [thetadot]])
        e_h, e_z = h_r - h, z_r - z
        int_h = self.int_h + P.Ts / 2 * (e_h + self.e_h_prev)
        int_z = self.int_z + P.Ts / 2 * (e_z + self.e_z_prev)
        u = self.forces(x_lon, x_lat, int_h, int_z)
        # anti-windup: hold both integrators while a rotor saturates
        if np.all(u >= 0) and np.all(u <= P.f_max):
            self.int_h, self.int_z = int_h, int_z
        else:
            u = self.forces(x_lon, x_lat, self.int_h, self.int_z)
        self.e_h_prev, self.e_z_prev = e_h, e_z
        return np.clip(u, 0, P.f_max)
`,
    f13: String.raw`import control as cnt

class Controller:
    def __init__(self):
        M = P.mc + 2 * P.mr
        J = P.Jc + 2 * P.mr * P.d**2
        a = P.mu / M
        A_lon = np.array([[0.0, 1.0], [0.0, 0.0]])
        B_lon = np.array([[0.0], [1 / M]])
        C_lon = np.array([[1.0, 0.0]])
        A_lat = np.array([[0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0], [0.0, -P.g, -a, 0.0], [0.0, 0.0, 0.0, 0.0]])
        B_lat = np.array([[0.0], [0.0], [0.0], [1 / J]])
        C_lat = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]])   # y_lat = (z, theta)
        # F.12 controller: integrators on h and z
        aug = lambda A, B, C: (np.block([[A, np.zeros((A.shape[0], 1))], [-C, np.zeros((1, 1))]]), np.vstack([B, [[0.0]]]))
        A1, B1 = aug(A_lon, B_lon, C_lon)
        zeta = 0.707
        pair = lambda wn: np.roots([1, 2 * zeta * wn, wn**2])
        wn_h, wn_th, wn_z = 2.2 / 8.0, 2.2 / 0.8, 2.2 / 8.0
        K1 = cnt.place(A1, B1, np.concatenate([pair(wn_h), [-0.4]]))
        self.K_lon, self.ki_lon = K1[:, 0:2], K1[0, 2]
        A1, B1 = aug(A_lat, B_lat, C_lat[0:1, :])
        K1 = cnt.place(A1, B1, np.concatenate([pair(wn_z), pair(wn_th), [-0.4]]))
        self.K_lat, self.ki_lat = K1[:, 0:4], K1[0, 4]
        # Observers, each pair 10x faster than its controller pair:
        # L = place(A^T, C^T, q)^T (Eq. 13.16); the lateral L is 4x2 (outputs z, theta).
        L_lon = cnt.place(A_lon.T, C_lon.T, pair(10 * wn_h)).T
        L_lat = cnt.place(A_lat.T, C_lat.T, np.concatenate([pair(10 * wn_z), pair(10 * wn_th)])).T
        self.obs = [(A_lon, B_lon, C_lon, L_lon), (A_lat, B_lat, C_lat, L_lat)]
        self.Fe = M * P.g
        self.mix = np.array([[1.0, 1.0], [P.d, -P.d]])      # (F, tau) = mix (f_r, f_l)
        self.unmix = np.linalg.inv(self.mix)
        self.xhat_lon = np.zeros((2, 1))
        self.xhat_lat = np.zeros((4, 1))
        self.u_prev = np.array([self.Fe / 2, self.Fe / 2])  # last rotor forces
        self.int_h = self.int_z = 0.0
        self.e_h_prev = self.e_z_prev = 0.0

    def update(self, r, y):
        h_r, z_r = r[0, 0], r[1, 0]
        # observers, driven by the last (saturated) input: F~ = F - F_e, tau
        F, tau = self.mix @ self.u_prev
        (A, B, C, L), (A2, B2, C2, L2) = self.obs
        self.xhat_lon = self.rk4(lambda x: A @ x + B * (F - self.Fe) + L @ (y[1:2] - C @ x), self.xhat_lon)
        self.xhat_lat = self.rk4(lambda x: A2 @ x + B2 * tau + L2 @ (y[[0, 2]] - C2 @ x), self.xhat_lat)
        x_lon, x_lat = self.xhat_lon, self.xhat_lat
        # F.12 control law on the estimates, with anti-windup
        e_h, e_z = h_r - x_lon[0, 0], z_r - x_lat[0, 0]
        int_h = self.int_h + P.Ts / 2 * (e_h + self.e_h_prev)
        int_z = self.int_z + P.Ts / 2 * (e_z + self.e_z_prev)
        u = self.forces(x_lon, x_lat, int_h, int_z)
        if np.all(u >= 0) and np.all(u <= P.f_max):
            self.int_h, self.int_z = int_h, int_z
        else:
            u = self.forces(x_lon, x_lat, self.int_h, self.int_z)
        self.e_h_prev, self.e_z_prev = e_h, e_z
        u = np.clip(u, 0, P.f_max)
        self.u_prev = u
        # x_hat in the order of the state: (z, h, theta, zdot, hdot, thetadot)
        x_hat = np.array([x_lat[0, 0], x_lon[0, 0], x_lat[1, 0], x_lat[2, 0], x_lon[1, 0], x_lat[3, 0]])
        return u, x_hat

    def forces(self, x_lon, x_lat, int_h, int_z):
        F = self.Fe - (self.K_lon @ x_lon)[0, 0] - self.ki_lon * int_h
        tau = -(self.K_lat @ x_lat)[0, 0] - self.ki_lat * int_z
        return self.unmix @ np.array([F, tau])

    def rk4(self, f, x):
        F1 = f(x); F2 = f(x + P.Ts / 2 * F1); F3 = f(x + P.Ts / 2 * F2); F4 = f(x + P.Ts * F3)
        return x + P.Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)
`,
    f14: String.raw`import control as cnt

class Controller:
    def __init__(self):
        M = P.mc + 2 * P.mr
        J = P.Jc + 2 * P.mr * P.d**2
        a = P.mu / M
        A_lon = np.array([[0.0, 1.0], [0.0, 0.0]])
        B_lon = np.array([[0.0], [1 / M]])
        C_lon = np.array([[1.0, 0.0]])
        A_lat = np.array([[0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0], [0.0, -P.g, -a, 0.0], [0.0, 0.0, 0.0, 0.0]])
        B_lat = np.array([[0.0], [0.0], [0.0], [1 / J]])
        C_lat = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]])   # y_lat = (z, theta)
        # F.12 controller: integrators on h and z
        aug = lambda A, B, C: (np.block([[A, np.zeros((A.shape[0], 1))], [-C, np.zeros((1, 1))]]), np.vstack([B, [[0.0]]]))
        A1, B1 = aug(A_lon, B_lon, C_lon)
        zeta = 0.707
        pair = lambda wn: np.roots([1, 2 * zeta * wn, wn**2])
        wn_h, wn_th, wn_z = 2.2 / 8.0, 2.2 / 0.8, 2.2 / 8.0
        K1 = cnt.place(A1, B1, np.concatenate([pair(wn_h), [-0.4]]))
        self.K_lon, self.ki_lon = K1[:, 0:2], K1[0, 2]
        A1, B1 = aug(A_lat, B_lat, C_lat[0:1, :])
        K1 = cnt.place(A1, B1, np.concatenate([pair(wn_z), pair(wn_th), [-0.4]]))
        self.K_lat, self.ki_lat = K1[:, 0:4], K1[0, 4]
        # Disturbance observers (Sec. 14.2): d_F at the force input; d_z, a force in
        # zddot (it also absorbs a wind speed), and d_tau at the torque input.
        Bd_lat = np.array([[0.0, 0.0], [0.0, 0.0], [1 / M, 0.0], [0.0, 1 / J]])
        A2_lon = np.block([[A_lon, B_lon], [np.zeros((1, 3))]])
        A2_lat = np.block([[A_lat, Bd_lat], [np.zeros((2, 6))]])
        B2_lon, B2_lat = np.vstack([B_lon, [[0.0]]]), np.vstack([B_lat, np.zeros((2, 1))])
        C2_lon, C2_lat = np.hstack([C_lon, [[0.0]]]), np.hstack([C_lat, np.zeros((2, 2))])
        L_lon = cnt.place(A2_lon.T, C2_lon.T, np.concatenate([pair(10 * wn_h), [-1.0]])).T
        L_lat = cnt.place(A2_lat.T, C2_lat.T, np.concatenate([pair(10 * wn_z), pair(10 * wn_th), [-1.0, -10.0]])).T
        self.obs = [(A2_lon, B2_lon, C2_lon, L_lon), (A2_lat, B2_lat, C2_lat, L_lat)]
        self.Fe = M * P.g
        self.mix = np.array([[1.0, 1.0], [P.d, -P.d]])      # (F, tau) = mix (f_r, f_l)
        self.unmix = np.linalg.inv(self.mix)
        self.xhat_lon = np.zeros((3, 1))   # (h, hdot, d_F)
        self.xhat_lat = np.zeros((6, 1))   # (z, theta, zdot, thetadot, d_z, d_tau)
        self.u_prev = np.array([self.Fe / 2, self.Fe / 2])  # last rotor forces
        self.int_h = self.int_z = 0.0
        self.e_h_prev = self.e_z_prev = 0.0

    def update(self, r, y):
        h_r, z_r = r[0, 0], r[1, 0]
        # observers, driven by the last (saturated) input: F~ = F - F_e, tau
        F, tau = self.mix @ self.u_prev
        (A, B, C, L), (A2, B2, C2, L2) = self.obs
        self.xhat_lon = self.rk4(lambda x: A @ x + B * (F - self.Fe) + L @ (y[1:2] - C @ x), self.xhat_lon)
        self.xhat_lat = self.rk4(lambda x: A2 @ x + B2 * tau + L2 @ (y[[0, 2]] - C2 @ x), self.xhat_lat)
        x_lon, x_lat = self.xhat_lon, self.xhat_lat
        # F.12 control law on the estimates, minus d_hat, with anti-windup
        e_h, e_z = h_r - x_lon[0, 0], z_r - x_lat[0, 0]
        int_h = self.int_h + P.Ts / 2 * (e_h + self.e_h_prev)
        int_z = self.int_z + P.Ts / 2 * (e_z + self.e_z_prev)
        u = self.forces(x_lon, x_lat, int_h, int_z)
        if np.all(u >= 0) and np.all(u <= P.f_max):
            self.int_h, self.int_z = int_h, int_z
        else:
            u = self.forces(x_lon, x_lat, self.int_h, self.int_z)
        self.e_h_prev, self.e_z_prev = e_h, e_z
        u = np.clip(u, 0, P.f_max)
        self.u_prev = u
        # x_hat in the order of the state: (z, h, theta, zdot, hdot, thetadot)
        x_hat = np.array([x_lat[0, 0], x_lon[0, 0], x_lat[1, 0], x_lat[2, 0], x_lon[1, 0], x_lat[3, 0]])
        d_hat = np.array([x_lon[2, 0], x_lat[4, 0], x_lat[5, 0]])   # (d_F, d_z, d_tau)
        return u, x_hat, d_hat

    def forces(self, x_lon, x_lat, int_h, int_z):
        # subtract the matched disturbance estimates (d_z is not matched to tau;
        # removing the estimator bias lets the z integrator cancel it)
        F = self.Fe - (self.K_lon @ x_lon[0:2])[0, 0] - self.ki_lon * int_h - x_lon[2, 0]
        tau = -(self.K_lat @ x_lat[0:4])[0, 0] - self.ki_lat * int_z - x_lat[5, 0]
        return self.unmix @ np.array([F, tau])

    def rk4(self, f, x):
        F1 = f(x); F2 = f(x + P.Ts / 2 * F1); F3 = f(x + P.Ts / 2 * F2); F4 = f(x + P.Ts * F3)
        return x + P.Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)
`,
  };

  // ----------------------------------------------------------- models --
  function sub(p) {
    const m = WB.systems.F.models(p);
    return {
      m,
      lon: m.lonSS,
      lat: m.latSS,
      Cz: [[1, 0, 0, 0]],
      // observer sub-blocks
      zBlk: { A: [[0, 1], [0, -m.a]], C: [[1, 0]] },
      tBlk: { A: [[0, 1], [0, 0]], C: [[1, 0]] },
      // disturbance-observer blocks: (h, ḣ, d_F), (z, ż, d_z), (θ, θ̇, d_τ)
      hD: { A: [[0, 1, 0], [0, 0, 1 / m.M], [0, 0, 0]], C: [[1, 0, 0]] },
      zD: { A: [[0, 1, 0], [0, -m.a, 1 / m.M], [0, 0, 0]], C: [[1, 0, 0]] },
      tD: { A: [[0, 1, 0], [0, 0, 1 / m.J], [0, 0, 0]], C: [[1, 0, 0]] },
    };
  }
  const augI = (A, B, Cr) => D().augmentIntegrator(A, B, Cr);
  const kr = (A, B, C, K) => D().refGain(A, B, C, K);

  // Desired poles from the knobs: lon pair; lateral = outer pair ∪ inner pair (F.11a).
  function ctrlPoles(k) {
    const wnh = 2.2 / k.trh, wnt = 2.2 / k.trth, wnz = 2.2 / (k.Msep * k.trth);
    return { lon: F.polesWZ(wnh, k.zetah), inner: F.polesWZ(wnt, k.zetath), outer: F.polesWZ(wnz, k.zetaz), wnh, wnt, wnz };
  }

  // Gains for a level: 'sf' (F.11), 'sfi' (F.12), 'obs' (F.13), 'dobs' (F.14).
  function design(p, k, level) {
    const S = sub(p);
    const P = ctrlPoles(k);
    const out = { poles: P };
    const latP = [...P.outer, ...P.inner];
    if (level === 'sf') {
      out.Kh = D().place(S.lon.A, S.lon.B, P.lon) || [NaN, NaN];
      out.Kz = D().place(S.lat.A, S.lat.B, latP) || [NaN, NaN, NaN, NaN];
      out.krh = kr(S.lon.A, S.lon.B, S.lon.C, out.Kh);
      out.krz = kr(S.lat.A, S.lat.B, S.Cz, out.Kz);
    } else {
      const ah = augI(S.lon.A, S.lon.B, S.lon.C), az = augI(S.lat.A, S.lat.B, S.Cz);
      const K1 = D().place(ah.A1, ah.B1, [...P.lon, { re: k.pIh, im: 0 }]) || [NaN, NaN, NaN];
      const K2 = D().place(az.A1, az.B1, [...latP, { re: k.pIz, im: 0 }]) || [NaN, NaN, NaN, NaN, NaN];
      out.Kh = K1.slice(0, 2); out.kIh = K1[2];
      out.Kz = K2.slice(0, 4); out.kIz = K2[4];
    }
    if (level === 'obs' || level === 'dobs') {
      const f = k.obsFactor, zo = k.zetaObs;
      const oh = F.polesWZ(f * P.wnh, zo), oz = F.polesWZ(f * P.wnz, zo), ot = F.polesWZ(f * P.wnt, zo);
      out.obsPoles = { lon: oh, outer: oz, inner: ot };
      if (level === 'obs') {
        out.Lh = col(D().observerGain(S.lon.A, S.lon.C, oh), 2);
        out.Lz = col(D().observerGain(S.zBlk.A, S.zBlk.C, oz), 2);
        out.Lt = col(D().observerGain(S.tBlk.A, S.tBlk.C, ot), 2);
      } else {
        out.obsPoles = { lon: [...oh, { re: k.pDh, im: 0 }], outer: [...oz, { re: k.pDz, im: 0 }], inner: [...ot, { re: k.pDth, im: 0 }] };
        out.Lh = col(D().observerGain(S.hD.A, S.hD.C, out.obsPoles.lon), 3);
        out.Lz = col(D().observerGain(S.zD.A, S.zD.C, out.obsPoles.outer), 3);
        out.Lt = col(D().observerGain(S.tD.A, S.tD.C, out.obsPoles.inner), 3);
      }
    }
    return out;
  }
  const col = (Lc, n) => (Lc ? Lc.map((r) => r[0]) : new Array(n).fill(NaN));
  // Full lateral L (4×2) from the block gains
  const latL = (Lz, Lt) => [[Lz[0], 0], [0, Lt[0]], [Lz[1], 0], [0, Lt[1]]];

  // ------------------------------------------------------- controller --
  function makeSS(ctx, { linear = false } = {}) {
    const s = ctx.sys, p = ctx.pModel, st = ctx.st, g = ctx.gains, level = ctx.chapter.level, Ts = ctx.S.sim.Ts;
    const S = sub(p), m = S.m;
    const Fe = F.feOf(ctx);   // the student's F_e in Work mode until F.4(a) is solved
    const [lo, hi] = s.uLimit(p)[0];
    const useObs = level === 'obs' || level === 'dobs';
    const useDO = level === 'dobs' && st.dobs !== false;
    const integ = level !== 'sf';
    let Ih = 0, Iz = 0, ehPrev = 0, ezPrev = 0;
    let xh = [0, 0, 0], xl = [0, 0, 0, 0, 0, 0];   // (h, ḣ, d_F), (z, θ, ż, θ̇, d_z, d_τ)
    let uPrev = null;
    const Lh = g.Lh || [0, 0, 0], Lz = g.Lz || [0, 0, 0], Lt = g.Lt || [0, 0, 0];
    const dot = (a, b) => a.reduce((acc, v, i) => acc + v * b[i], 0);

    function obsStep(y) {
      // previous input as applied: saturated rotor forces → F̃, τ
      let Ft = 0, tau = 0;
      if (uPrev) {
        const u = linear ? uPrev : uPrev.map((v) => Math.max(lo, Math.min(hi, v)));
        Ft = u[0] + u[1] - (linear ? 0 : Fe);
        tau = p.d * (u[0] - u[1]);
      }
      const fh = (x) => {
        const e = y[1] - x[0];
        return [x[1] + Lh[0] * e, (Ft + (useDO ? x[2] : 0)) / m.M + Lh[1] * e, useDO ? Lh[2] * e : 0];
      };
      const fl = (x) => {
        const ez = y[0] - x[0], et = y[2] - x[1];
        return [
          x[2] + Lz[0] * ez,
          x[3] + Lt[0] * et,
          -p.g * x[1] - m.a * x[2] + (useDO ? x[4] / m.M : 0) + Lz[1] * ez,
          (tau + (useDO ? x[5] : 0)) / m.J + Lt[1] * et,
          useDO ? Lz[2] * ez : 0,
          useDO ? Lt[2] * et : 0,
        ];
      };
      xh = M.rk4Step(fh, xh, 0, Ts);
      xl = M.rk4Step(fl, xl, 0, Ts);
    }

    return {
      update(r, x, y) {
        let xlon, xlat;
        if (useObs) {
          obsStep(y);
          xlon = [xh[0], xh[1]]; xlat = xl.slice(0, 4);
        } else {
          xlon = [x[1], x[4]]; xlat = [x[0], x[2], x[3], x[5]];
        }
        const compute = (IhT, IzT) => {
          let Ft, tau;
          if (integ) {
            Ft = -dot(g.Kh, xlon) - g.kIh * IhT;
            tau = -dot(g.Kz, xlat) - g.kIz * IzT;
          } else {
            Ft = -dot(g.Kh, xlon) + g.krh * r[0];
            tau = -dot(g.Kz, xlat) + g.krz * r[1];
          }
          if (useDO) { Ft -= xh[2]; tau -= xl[5]; }
          const Fc = linear ? Ft : Fe + Ft;
          return { u: s.mix(Fc, tau, p), Ft, tau };
        };
        let out;
        if (integ) {
          const eh = r[0] - xlon[0], ez = r[1] - xlat[0];
          const IhT = Ih + (Ts / 2) * (eh + ehPrev), IzT = Iz + (Ts / 2) * (ez + ezPrev);
          out = compute(IhT, IzT);
          const sat = !linear && out.u.some((v) => v > hi || v < lo);
          if (st.antiwindup === 'clamp' && sat) out = compute(Ih, Iz);   // hold the integrators (F.12a)
          else { Ih = IhT; Iz = IzT; }
          ehPrev = eh; ezPrev = ez;
        } else {
          out = compute(0, 0);
        }
        uPrev = out.u;
        return {
          u: out.u, Ftilde: out.Ft, tau: out.tau, intH: Ih, intZ: Iz,
          ...(useObs ? { zhat: xl[0], hhat: xh[0], thhat: xl[1], zdhat: xl[2], hdhat: xh[1], thdhat: xl[3], dFhat: xh[2], dzhat: xl[4], dthat: xl[5] } : {}),
        };
      },
    };
  }

  // ------------------------------------------------------------ poles --
  function clPoles(ctx, level, g = ctx.gains) {
    const S = sub(ctx.pModel);
    if (level === 'sf') {
      return {
        lon: L.eig(L.sub(S.lon.A, L.mul(S.lon.B, [g.Kh]))),
        lat: L.eig(L.sub(S.lat.A, L.mul(S.lat.B, [g.Kz]))),
      };
    }
    const ah = augI(S.lon.A, S.lon.B, S.lon.C), az = augI(S.lat.A, S.lat.B, S.Cz);
    return {
      lon: L.eig(L.sub(ah.A1, L.mul(ah.B1, [[...g.Kh, g.kIh]]))),
      lat: L.eig(L.sub(az.A1, L.mul(az.B1, [[...g.Kz, g.kIz]]))),
    };
  }
  function obsPolesOf(ctx, level, g = ctx.gains) {
    const S = sub(ctx.pModel);
    if (level === 'obs') {
      return {
        lon: L.eig(L.sub(S.lon.A, L.mul(L.col(g.Lh.slice(0, 2)), S.lon.C))),
        lat: L.eig(L.sub(S.lat.A, L.mul(latL(g.Lz, g.Lt), S.lat.C))),
      };
    }
    if (level === 'dobs') {
      const blk = (B, Lv) => L.eig(L.sub(B.A, L.mul(L.col(Lv), B.C)));
      return { lon: blk(S.hD, g.Lh), lat: [...blk(S.zD, g.Lz), ...blk(S.tD, g.Lt)] };
    }
    return { lon: [], lat: [] };
  }

  // ----------------------------------------------------------- controls --
  const KEYS = {
    sf: { lon: ['Kh1', 'Kh2', 'krh'], lat: ['Kz1', 'Kz2', 'Kz3', 'Kz4', 'krz'] },
    sfi: { lon: ['Kh1', 'Kh2', 'kIh'], lat: ['Kz1', 'Kz2', 'Kz3', 'Kz4', 'kIz'] },
  };
  // Work-mode gains (flat st.w) → gain object
  function fromW(w, level) {
    const g = { Kh: [w.Kh1, w.Kh2], Kz: [w.Kz1, w.Kz2, w.Kz3, w.Kz4], krh: w.krh, krz: w.krz, kIh: w.kIh, kIz: w.kIz };
    if (level === 'obs') { g.Lh = [w.Lh1, w.Lh2, 0]; g.Lz = [w.Lz1, w.Lz2, 0]; g.Lt = [w.Lt1, w.Lt2, 0]; }
    if (level === 'dobs') { g.Lh = [w.Lh1, w.Lh2, w.Ldh]; g.Lz = [w.Lz1, w.Lz2, w.Ldz]; g.Lt = [w.Lt1, w.Lt2, w.Ldt]; }
    return g;
  }
  function toW(g) {
    const w = { Kh1: g.Kh[0], Kh2: g.Kh[1], Kz1: g.Kz[0], Kz2: g.Kz[1], Kz3: g.Kz[2], Kz4: g.Kz[3] };
    for (const k of ['krh', 'krz', 'kIh', 'kIz']) if (g[k] !== undefined) w[k] = g[k];
    if (g.Lh) Object.assign(w, { Lh1: g.Lh[0], Lh2: g.Lh[1], Lz1: g.Lz[0], Lz2: g.Lz[1], Lt1: g.Lt[0], Lt2: g.Lt[1] });
    if (g.Lh && g.Lh.length > 2) Object.assign(w, { Ldh: g.Lh[2], Ldz: g.Lz[2], Ldt: g.Lt[2] });
    return w;
  }
  // Work-mode starting gains: a deliberately slow design (t_r,h = 16 s, M = 20,
  // t_r,θ = 1.2 s, ζ = 0.9, observers 3× faster): stable, but not the answer. The
  // reference gains start at 0: the designed k_r would show its form (k_r = K_1 here).
  function slowW(sys, level) {
    const p = { ...Object.fromEntries(sys.params.map((q) => [q.key, q.value])), ...sys.constants };
    const k = { trh: 16, zetah: 0.9, trth: 1.2, zetath: 0.9, Msep: 20, zetaz: 0.9, pIh: -0.05, pIz: -0.03, obsFactor: 3, zetaObs: 0.9, pDh: -0.5, pDz: -0.5, pDth: -3 };
    return { ...toW(design(p, k, level)), krh: 0, krz: 0 };
  }
  function knobs(sys, extra = {}) {
    const pr = sys.problems.ch8;
    return { trh: pr.trh, zetah: pr.zetah, trth: pr.trth, zetath: pr.zetath, Msep: pr.Msep, zetaz: pr.zetaz,
      pIh: sys.problems.ch12.pIh, pIz: sys.problems.ch12.pIz, obsFactor: sys.problems.ch13.obsFactor, zetaObs: 0.707,
      pDh: sys.problems.ch14.pDh, pDz: sys.problems.ch14.pDz, pDth: sys.problems.ch14.pDth, ...extra };
  }
  function knobControls(parent, ctx, level, title) {
    const k = ctx.st.k;
    const s1 = section(parent, `${title}: controller poles`, 'p. 113 · Eq. 8.5, F.11(a) p. 399');
    slider(s1, { label: 't<sub>r,h</sub>', unit: 's', min: 0.5, max: 20, step: 0.01, sig: 3, ...bind(ctx, 'trh', () => k) });
    slider(s1, { label: 'ζ<sub>h</sub>', min: 0.3, max: 1.5, step: 0.005, sig: 3, ...bind(ctx, 'zetah', () => k) });
    slider(s1, { label: 't<sub>r,θ</sub>', unit: 's', min: 0.1, max: 3, step: 0.005, sig: 3, ...bind(ctx, 'trth', () => k) });
    slider(s1, { label: 'ζ<sub>θ</sub>', min: 0.3, max: 1.5, step: 0.005, sig: 3, ...bind(ctx, 'zetath', () => k) });
    slider(s1, { label: 'M = t<sub>r,z</sub>/t<sub>r,θ</sub>', min: 1, max: 30, step: 0.1, sig: 3, ...bind(ctx, 'Msep', () => k) });
    slider(s1, { label: 'ζ<sub>z</sub>', min: 0.3, max: 1.5, step: 0.005, sig: 3, ...bind(ctx, 'zetaz', () => k) });
    if (level !== 'sf') {
      slider(s1, { label: 'p<sub>I,h</sub>', min: -3, max: -0.01, step: 0.001, sig: 3, ...bind(ctx, 'pIh', () => k) });
      slider(s1, { label: 'p<sub>I,z</sub>', min: -3, max: -0.01, step: 0.001, sig: 3, ...bind(ctx, 'pIz', () => k) });
    }
    if (level === 'obs' || level === 'dobs') {
      const s2 = section(parent, `${title}: observer poles`, 'p. 222');
      slider(s2, { label: 'ω<sub>n,obs</sub> / ω<sub>n,ctrl</sub>', min: 1, max: 30, step: 0.1, sig: 3, ...bind(ctx, 'obsFactor', () => k), hint: 'each observer pair is this many times faster than its controller pair' });
      slider(s2, { label: 'ζ<sub>obs</sub>', min: 0.3, max: 1.5, step: 0.005, sig: 3, ...bind(ctx, 'zetaObs', () => k) });
      if (level === 'dobs') {
        slider(s2, { label: 'p<sub>d,h</sub>', min: -20, max: -0.05, step: 0.01, sig: 3, ...bind(ctx, 'pDh', () => k) });
        slider(s2, { label: 'p<sub>d,z</sub>', min: -20, max: -0.05, step: 0.01, sig: 3, ...bind(ctx, 'pDz', () => k) });
        slider(s2, { label: 'p<sub>d,θ</sub>', min: -60, max: -0.1, step: 0.01, sig: 3, ...bind(ctx, 'pDth', () => k) });
      }
    }
  }
  function gainReadout(parent, ctx, level) {
    WB.ui.readout(parent, () => {
      const w = toW(ctx.gains);
      const keys = [...KEYS[level === 'sf' ? 'sf' : 'sfi'].lon, ...KEYS[level === 'sf' ? 'sf' : 'sfi'].lat,
        ...(level === 'obs' || level === 'dobs' ? ['Lh1', 'Lh2', 'Lz1', 'Lz2', 'Lt1', 'Lt2'] : []), ...(level === 'dobs' ? ['Ldh', 'Ldz', 'Ldt'] : [])];
      return keys.map((k) => [k, w[k]]);
    });
  }

  // ------------------------------------------------------------- s-plane --
  function splane(ctx) {
    const view = ctx.st.view === 'lon' ? 'lon' : 'lat';
    const S = sub(ctx.pModel);
    const cl = clPoles(ctx, this.level), ob = obsPolesOf(ctx, this.level);
    const mk = [];
    const ol = view === 'lon' ? L.eig(S.lon.A) : L.eig(S.lat.A);
    if (F.showsAnswer(ctx, olKey(view))) ol.forEach((q) => mk.push({ ...q, kind: 'ol', label: 'open-loop pole' }));
    // Work mode has no gains: the plots run the student's own controller.
    if (ctx.S.mode === 'explore') {
      cl[view].forEach((q) => mk.push({ ...q, kind: 'cl', label: `controller pole (${view === 'lon' ? 'altitude' : 'lateral'})` }));
      ob[view].forEach((q) => mk.push({ ...q, kind: 'obs', label: 'observer pole', noFit: Math.hypot(q.re, q.im) > 8 }));
    }
    if (ctx.S.mode === 'work' && ctx.app.isSolved('F.11/a')) {
      const P = ctrlPoles(ctx.st.k);
      (view === 'lon' ? P.lon : [...P.outer, ...P.inner]).forEach((q) => mk.push({ ...q, kind: 'target', label: 'target pole (spec)' }));
    }
    return { markers: mk, zetaRay: view === 'lon' ? ctx.st.k.zetah : ctx.st.k.zetaz, minR: 0.05 };
  }

  // ----------------------------------------------------------- math cards --
  function ssCards(ctx) {
    const S = sub(ctx.pModel);
    return [
      { title: 'Decoupled models (F.6)', page: 'F.6 p. 396', answers: ['F.6/a', 'F.6/b'],
        theory: 'x_{lon} = (h, \\dot h),\\; u = \\tilde F;\\quad x_{lat} = (z, \\theta, \\dot z, \\dot\\theta),\\; u = \\tilde\\tau',
        numbers: `A_{lon} = ${texMat(S.lon.A)},\\; B_{lon} = ${texMat(S.lon.B)},\\quad A_{lat} = ${texMat(S.lat.A)},\\; B_{lat} = ${texMat(S.lat.B)}` },
    ];
  }
  // The controllability matrices are built from the F.6 models (and answer F.11(c)).
  const ctrbCard = (A, B, title, page, answers) => ({ ...WB.ss.ctrbCard(A, B, title, page, { sig: 3 }), answers });
  // In Work mode the open-loop poles (eig A, F.6) and the target rings (F.11(a)) stay off until solved.
  const olKey = (view) => (view === 'lon' ? 'F.6/a' : 'F.6/b');
  function polesCard(ctx, d, level) {
    const P = d.poles;
    const lat = [...P.outer, ...P.inner];
    return {
      title: 'Desired closed-loop poles', page: 'F.11(a) p. 399', answers: 'F.11/a',
      theory: '\\Delta^d_{lon} = s^2 + 2\\zeta_h\\omega_{n_h}s + \\omega_{n_h}^2,\\quad \\Delta^d_{lat} = (s^2 + 2\\zeta_z\\omega_{n_z}s + \\omega_{n_z}^2)(s^2 + 2\\zeta_\\theta\\omega_{n_\\theta}s + \\omega_{n_\\theta}^2)' + (level === 'sf' ? '' : '\\,(s - p_I)'),
      numbers: `p_{lon} = ${P.lon.map((q) => texPole(q)).join(',\\;')},\\quad p_{lat} = ${lat.map((q) => texPole(q)).join(',\\;')}`,
      note: 'The F.8 pairs: the outer pair is the slow lateral mode, the inner pair the fast roll mode.',
    };
  }

  // -------------------------------------------------------- chapter base --
  function base(level, num, title, pages, extra) {
    return F.register(Object.assign({
      id: `ch${num}`, num, tab: `Ch ${num}`, title, pages, level,
      controller(ctx, o) { return makeSS(ctx, o); },
      // Work mode simulates the student's controller (WB.myCtrl): F.11 and F.12 feed
      // back the state, as in the book; F.13 and F.14 get only y = (z, h, θ).
      implement: { feed: level === 'sf' || level === 'sfi' ? 'state' : 'y', linear: false },
      gains(ctx) { return ctx.S.mode === 'explore' ? design(ctx.pModel, ctx.st.k, level) : fromW(ctx.st.w, level); },
      splane,
      targets(ctx) { return { tr: ctx.st.k.trh }; },
    }, extra));
  }
  function viewSeg(parent, ctx) {
    segmented(parent, { label: 's-plane shows', options: [{ value: 'lon', label: 'altitude' }, { value: 'lat', label: 'lateral' }], ...bind(ctx, 'view') });
  }
  // Work-mode control panel: the plots show the student's controller, named by `part`.
  function workControls(parent, ctx, part, extra) {
    const sec = section(parent, 'Plots', 'F.11–F.14');
    viewSeg(sec, ctx);
    if (extra) segmented(sec, { label: 'Extra plot', options: extra, ...bind(ctx, 'extra') });
    F.workBanner(parent, ctx, part);
  }

  // The workbench's controller at `level` with the reference design on a check run
  // (the reference for WB.myCtrl checks).
  const LEVEL_CH = { sf: 'ch11', sfi: 'ch12', obs: 'ch13', dobs: 'ch14' };
  function refSS(ctx, sc, level, st = {}) {
    const rc = WB.myCtrl.refCtx(ctx, sc, { chapter: WB.studies.F.chapters[LEVEL_CH[level]] });
    rc.st = { ...ctx.st, antiwindup: 'clamp', dobs: true, ...st };
    rc.gains = design(sc.params, knobs(ctx.sys), level);
    return WB.myCtrl.reference(ctx, sc, makeSS(rc));
  }
  const STEP = (amplitude) => ({ type: 'step', amplitude, tStep: 0 });
  // F.11(e): the 10–90% rise time of ζ = 1 poles at ω_n = 2.2/8 (the slowest the
  // F.11(a) inequalities allow) is 3.36/ω_n = 12.2 s.
  const TR11 = 12.5;
  // F.12(a) windup test: f_max lowered to just above hover, so a 10 m climb keeps a
  // rotor at its limit for seconds and an integrator that keeps integrating winds up.
  const WINDUP = { fmax: 7.45, step: 10, os: 0.5, tEnd: 30 };
  const misText = (ctx, mis) => Object.entries(mis).map(([k, v]) => `${ctx.sys.params.find((q) => q.key === k).label} ${v > 0 ? '+' : ''}${v}%`).join(', ');
  const fm = (v) => `${fmt(v, 3)} m`;

  // Satisfies "ζ ≥ ζ_spec and ω_n ≥ ω_n,spec" (F.11a) for every pole.
  function meetsSpec(poles, wn, zeta) {
    return poles.every((q) => {
      const w = Math.hypot(q.re, q.im);
      const z = w > 0 ? -q.re / w : 1;
      return w >= wn * 0.99 && z >= zeta - 0.005;
    });
  }

  // ------------------------------------------------------------ Chapter 11 --
  base('sf', 11, 'Full state feedback', 'pp. 173–196, F.11 p. 399', {
    defaults(sys) { return { view: 'lat', zOff: 3, hOff: 0, k: knobs(sys), w: slowW(sys, 'sf') }; },
    simDefaults(sys) { return sys.problems.ch11.sim; },
    buildControls(parent, ctx) {
      if (ctx.S.mode === 'work') { workControls(parent, ctx, 'F.11(e)'); return; }
      const sec = section(parent, 'u = −Kx + k_r r (per loop)', 'p. 183 · Eq. 11.38');
      viewSeg(sec, ctx);
      sec.append(el('p', { class: 'muted small', text: 'F = F_e − K_h(h, ḣ) + k_r,h h_r;  τ = −K_z(z, θ, ż, θ̇) + k_r,z z_r. The true state is fed back.' }));
      knobControls(parent, ctx, 'sf', 'Design'); gainReadout(section(parent, 'Gains', 'p. 182'), ctx, 'sf');
    },
    math(ctx) {
      const S = sub(ctx.pModel), d = design(ctx.pModel, ctx.st.k, 'sf');
      return [
        ...ssCards(ctx),
        { title: 'Controllability', page: 'p. 180 · Eq. 11.29',
          theory: '\\mathcal{C}_{A,B} = \\begin{bmatrix} B & AB & \\cdots & A^{n-1}B\\end{bmatrix},\\quad \\text{controllable} \\iff \\operatorname{rank}\\mathcal{C}_{A,B} = n' },
        ctrbCard(S.lon.A, S.lon.B, 'Controllability (altitude)', 'F.11(c) p. 399', ['F.6/a', 'F.11/c']),
        ctrbCard(S.lat.A, S.lat.B, 'Controllability (lateral)', 'F.11(c) p. 399', ['F.6/b', 'F.11/c']),
        polesCard(ctx, d, 'sf'),
        { title: 'Gains (place / Ackermann)', page: 'p. 182 · Eq. 11.32, 11.35',
          theory: 'K = \\text{place}(A, B, p),\\quad k_r = \\frac{-1}{C_r(A - BK)^{-1}B}' },
        { title: 'Gains for the desired poles', page: 'F.11(d) p. 399', answers: ['F.11/d', 'F.11/d2'],
          theory: '\\text{each loop has a free integrator, so } k_{r,h} = K_{h,1},\\; k_{r,z} = K_{z,1}',
          numbers: `K_h = ${texMat([d.Kh])},\\; k_{r,h} = ${tex(d.krh)},\\quad K_z = ${texMat([d.Kz])},\\; k_{r,z} = ${tex(d.krz)}` },
        { title: 'Tuning (F.11e)', page: 'p. 110–113', answers: 'F.11/e',
          theory: 't_r \\approx \\frac{2.2}{\\omega_n}:\\; \\text{move poles farther from the origin to speed up};\\quad M_p = e^{-\\zeta\\pi/\\sqrt{1-\\zeta^2}}:\\; \\text{raise } \\zeta \\text{ (smaller angle from the real axis) to cut overshoot}' },
      ];
    },
    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch11;
      const S = () => sub(ctx.pModel);
      const P = () => ctrlPoles(knobs(ctx.sys));
      const ref = () => design(ctx.pModel, knobs(ctx.sys), 'sf');
      const num = (v) => PD().num(v);
      PD().problemPanel(parent, ctx, prob, [
        {
          id: 'a', title: '(a) Closed-loop pole locations',
          html: 'Using ω<sub>n<sub>h</sub></sub>, ζ<sub>h</sub>, ω<sub>n<sub>z</sub></sub>, ζ<sub>z</sub> from F.8, choose two longitudinal poles with damping ratios above ζ<sub>h</sub> and natural frequencies above ω<sub>n<sub>h</sub></sub>, and four lateral poles with damping ratios above ζ<sub>z</sub> and natural frequencies above ω<sub>n<sub>z</sub></sub>. Complex values are fine (<code>-1+2j</code>); give complex poles with their conjugates.',
          inputs: { p1: 'p<sub>lon,1</sub>', p2: 'p<sub>lon,2</sub>', q1: 'p<sub>lat,1</sub>', q2: 'p<sub>lat,2</sub>', q3: 'p<sub>lat,3</sub>', q4: 'p<sub>lat,4</sub>' },
          check: (v) => {
            const lon = [v.p1, v.p2].map(M.parseComplex), lat = [v.q1, v.q2, v.q3, v.q4].map(M.parseComplex);
            if ([...lon, ...lat].some((q) => !q)) return { ok: false, msg: 'Enter all six poles.' };
            const conj = (ps) => ps.every((q) => Math.abs(q.im) < 1e-9 || ps.some((r) => Math.abs(r.re - q.re) < 1e-6 && Math.abs(r.im + q.im) < 1e-6));
            if (!conj(lon) || !conj(lat)) return { ok: false, msg: 'Complex poles come in conjugate pairs (K is real).' };
            if (!meetsSpec(lon, P().wnh, prob.zetah || 0.707)) return { ok: false, msg: 'A longitudinal pole misses the ζ_h / ω_n,h spec (find both from F.8).' };
            if (!meetsSpec(lat, P().wnz, 0.707)) return { ok: false, msg: 'A lateral pole misses the ζ_z / ω_n,z spec (find both from F.8).' };
            return { ok: true, msg: 'All six poles meet the specs.' };
          },
          solution: () => [
            { tex: `\\omega_{n_h} = \\frac{2.2}{8} = ${tex(P().wnh)},\\; \\omega_{n_z} = \\frac{2.2}{8} = ${tex(P().wnz)},\\; \\zeta_h = \\zeta_z = 0.707` },
            { tex: `\\text{e.g. the F.8 pairs: } p_{lon} = ${P().lon.map((q) => texPole(q, 4)).join(',\\;')},\\quad p_{lat} = ${[...P().outer, ...P().inner].map((q) => texPole(q, 4)).join(',\\;')}` },
            { html: 'The lateral set is the F.8 outer pair (the slow position mode) plus the inner pair (the fast roll mode). Any poles that are faster and better damped also pass.' },
          ],
        },
        {
          id: 'b', title: '(b) State-space matrices from F.6',
          html: 'Add your A, B, C, D from F.6 (Ch 6 tab) to your param file. The model cards in the Math section unlock once F.6 is solved.',
        },
        {
          id: 'c', title: '(c) Controllability ranks', inputs: { rl: 'rank 𝒞 (altitude)', rz: 'rank 𝒞 (lateral)' },
          check: (v) => PD().checkNumbers(v, { rl: L.rank(L.ctrb(S().lon.A, S().lon.B)), rz: L.rank(L.ctrb(S().lat.A, S().lat.B)) }, {}),
          solution: () => [{ tex: `\\mathcal{C}_{lon} = ${texMat(L.ctrb(S().lon.A, S().lon.B))},\\quad \\mathcal{C}_{lat} = ${texMat(L.ctrb(S().lat.A, S().lat.B), 3)}` }, { html: 'Both full rank (2 and 4): each loop is controllable from its own input.' }],
        },
        {
          id: 'd', title: '(d) Altitude: K<sub>h</sub> and k<sub>r<sub>h</sub></sub>',
          html: 'Any K whose poles meet the longitudinal spec of (a) passes; k<sub>r<sub>h</sub></sub> must give unity DC gain from h<sub>r</sub> to h for your K.',
          inputs: { K1: 'K<sub>h,1</sub>', K2: 'K<sub>h,2</sub>', kr: 'k<sub>r,h</sub>' },
          check: (v) => {
            const K = [num(v.K1), num(v.K2)], k = num(v.kr);
            if (K.some((x) => x === null) || k === null) return { ok: false, msg: 'Fill in K and k_r.' };
            const poles = L.eig(L.sub(S().lon.A, L.mul(S().lon.B, [K])));
            if (!meetsSpec(poles, P().wnh, prob.zetah || 0.707)) return { ok: false, msg: `Poles ${poles.map((q) => fmtPole(q)).join(', ')} miss the ζ/ωn spec.` };
            const want = kr(S().lon.A, S().lon.B, S().lon.C, K);
            return M.close(k, want) ? { ok: true, msg: `Poles ${poles.map((q) => fmtPole(q)).join(', ')}.` } : { ok: false, msg: 'K is fine; check k_r.' };
          },
          solution: () => { const r = ref(); return [
            { tex: `p = ${P().lon.map((q) => texPole(q, 4)).join(',\\;')}\\;(\\text{the F.8 pair}):\\quad K_h = ${texMat([r.Kh])},\\quad k_{r,h} = ${tex(r.krh)}` },
            { html: 'With x = (h, ḣ), −K<sub>h</sub>x + k<sub>r,h</sub>h<sub>r</sub> is exactly the F.8 PD law with derivative on h: K<sub>h</sub> = (k<sub>P<sub>h</sub></sub>, k<sub>D<sub>h</sub></sub>).' },
          ]; },
        },
        {
          id: 'd2', title: '(d) Lateral: K<sub>z</sub> and k<sub>r<sub>z</sub></sub>',
          html: 'Any K whose four poles meet the lateral spec of (a) passes; k<sub>r<sub>z</sub></sub> must give unity DC gain from z<sub>r</sub> to z for your K.',
          inputs: { K1: 'K<sub>z,1</sub>', K2: 'K<sub>z,2</sub>', K3: 'K<sub>z,3</sub>', K4: 'K<sub>z,4</sub>', kr: 'k<sub>r,z</sub>' },
          check: (v) => {
            const K = [num(v.K1), num(v.K2), num(v.K3), num(v.K4)], k = num(v.kr);
            if (K.some((x) => x === null) || k === null) return { ok: false, msg: 'Fill in K and k_r.' };
            const poles = L.eig(L.sub(S().lat.A, L.mul(S().lat.B, [K])));
            if (!meetsSpec(poles, P().wnz, 0.707)) return { ok: false, msg: `Poles ${poles.map((q) => fmtPole(q)).join(', ')} miss the ζ/ωn spec.` };
            const want = kr(S().lat.A, S().lat.B, S().Cz, K);
            return M.close(k, want) ? { ok: true, msg: `Poles ${poles.map((q) => fmtPole(q)).join(', ')}.` } : { ok: false, msg: 'K is fine; check k_r.' };
          },
          solution: () => { const r = ref(); return [
            { tex: `p = ${[...P().outer, ...P().inner].map((q) => texPole(q, 4)).join(',\\;')}` },
            { tex: `\\Delta^d = ${T.polyTex(L.polyFromRoots([...P().outer, ...P().inner]))}` },
            { tex: `K_z = ${texMat([r.Kz])},\\quad k_{r,z} = ${tex(r.krz)}` },
            { html: 'One reasonable choice: the F.8 outer and inner pairs (the problem allows anything faster and better damped). K<sub>z,1</sub> and K<sub>z,3</sub> are negative for the same reason k<sub>P<sub>z</sub></sub>, k<sub>D<sub>z</sub></sub> are.' },
          ]; },
        },
        WB.myCtrl.part(ctx, {
          id: 'e', title: '(e) Implement the state-feedback controller and tune it', seed: ['F.10/b'],
          html: `Compute your gains from (d) in <code>__init__</code> (from <code>P</code>). <code>update</code> gets r = [[h<sub>r</sub>], [z<sub>r</sub>]] and the state x, and returns [f<sub>r</sub>, f<sub>ℓ</sub>]. The check runs 2 m steps in h<sub>r</sub> and z<sub>r</sub> for 30 s, with the nominal and with other parameters: for both h and z the rise time must be at most ${TR11} s (the slowest response the (a) specs allow), the overshoot at most 6%, and the error at the end under 2 cm. Which pole changes reduce the rise time, and which the overshoot? Try them here.`,
          check: async (code) => {
            let first = '';
            for (const pc of WB.myCtrl.paramCases(ctx)) {
              const sc = F.scenario(ctx, { params: pc.params, refs: [STEP(2), STEP(2)], tEnd: 30, feed: 'state' });
              const res = await WB.myCtrl.run(ctx, code, sc);
              if (res.ok === false) return res;
              const n = res.t.length, bad = [], txt = [];
              for (const [oi, name] of [[1, 'h'], [0, 'z']]) {
                const y = res.yAll[oi], m = M.stepMetrics(res.t, y, 0, n, y[0], 2), e = Math.abs(F.endErr(res, oi));
                txt.push(`${name}: rise ${fmt(m.tr, 3)} s, overshoot ${fmt(m.os, 3)}%, end error ${fm(e)}`);
                if (!(m.tr <= TR11)) bad.push(`${name} rises too slowly`);
                if (!(m.os <= 6)) bad.push(`${name} overshoots too much`);
                if (!(e < 0.02)) bad.push(`${name} does not settle on its reference (DC gain)`);
              }
              if (bad.length) return { ok: false, msg: `With ${pc.label}: ${bad.join(', ')}. ${txt.join('; ')}.` };
              first = first || txt.join('; ');
            }
            return { ok: true, msg: `Meets the specs with the nominal and other parameters (nominal: ${first}).` };
          },
          solution: () => [
            { code: SOL.f11 },
            { html: 'With x = (h, ḣ), −K<sub>h</sub>x + k<sub>r,h</sub>h<sub>r</sub> is exactly the F.8 altitude PD; the lateral K<sub>z</sub> places the F.8 outer and inner pairs on the full 4-state model. Rise time: move the dominant poles farther from the origin (larger ω<sub>n</sub>). Overshoot: increase ζ (move the poles toward the real axis). Faster poles need larger rotor forces, so watch the limits.' },
          ],
        }),
      ]);
    },
  });

  // ------------------------------------------------------------ Chapter 12 --
  base('sfi', 12, 'Integrators with state feedback', 'pp. 197–214, F.12 p. 400', {
    defaults(sys) { return { view: 'lat', zOff: 3, hOff: 0, antiwindup: 'clamp', extra: 'int', k: knobs(sys), w: slowW(sys, 'sfi') }; },
    simDefaults(sys) { const pr = sys.problems.ch12; return { ...pr.sim, refs: [{ type: 'step', amplitude: 2.5 }], dists: [0, pr.dists.Fwind], mismatch: pr.mismatch }; },
    buildControls(parent, ctx) {
      if (ctx.S.mode === 'work') { workControls(parent, ctx, 'F.12(a) or (c)'); return; }
      const sec = section(parent, 'u = −Kx − k_I ∫(r − y)', 'p. 199');
      viewSeg(sec, ctx);
      segmented(sec, { label: 'Anti-windup (F.12a)', options: [{ value: 'clamp', label: 'hold integrators while a rotor saturates' }, { value: 'none', label: 'none' }], ...bind(ctx, 'antiwindup') });
      knobControls(parent, ctx, 'sfi', 'Design'); gainReadout(section(parent, 'Gains', 'p. 199'), ctx, 'sfi');
    },
    extraPlot(ctx, res) {
      if (ctx.S.mode === 'work') return null;  // the student's controller reports no internals
      return { opts: { title: 'integrators', yLabel: 'x_I [m·s]', unit: 'm·s' }, data: { series: [
        { label: 'x_I,h = ∫(h_r − h)', y: Array.from(res.extras.intH || []), color: '--series-1' },
        { label: 'x_I,z = ∫(z_r − z)', y: Array.from(res.extras.intZ || []), color: '--series-3' },
      ] } };
    },
    math(ctx) {
      const S = sub(ctx.pModel), d = design(ctx.pModel, ctx.st.k, 'sfi');
      const ah = augI(S.lon.A, S.lon.B, S.lon.C), az = augI(S.lat.A, S.lat.B, S.Cz);
      return [
        { title: 'Augmented systems', page: 'p. 198 · Eq. 12.1',
          theory: '\\dot x_I = r - C_r x,\\quad A_1 = \\begin{bmatrix}A & 0\\\\ -C_r & 0\\end{bmatrix},\\quad B_1 = \\begin{bmatrix}B\\\\0\\end{bmatrix}' },
        { title: 'Augmented VTOL models', page: 'F.6 p. 396', answers: ['F.6/a', 'F.6/b'],
          numbers: `A_{1,lon} = ${texMat(ah.A1)},\\quad A_{1,lat} = ${texMat(az.A1, 3)}` },
        ctrbCard(az.A1, az.B1, 'Controllability of (A₁, B₁), lateral', 'p. 198', 'F.6/b'),
        polesCard(ctx, d, 'sfi'),
        { title: 'Gains', page: 'p. 199–201',
          theory: '\\begin{bmatrix}K & k_I\\end{bmatrix} = \\text{place}(A_1, B_1, p),\\quad u = -Kx - k_I x_I' },
        { title: 'Gains for the desired poles', page: 'F.12(a) p. 400', answers: 'F.12/a',
          numbers: `K_h = ${texMat([d.Kh])},\\; k_{I,h} = ${tex(d.kIh)},\\quad K_z = ${texMat([d.Kz])},\\; k_{I,z} = ${tex(d.kIz)}` },
        { title: 'Wind as a lateral force', page: 'F.12(b) p. 400',
          theory: '\\ddot z = \\frac{-(f_r + f_\\ell)\\sin\\theta - \\mu\\dot z + F_{wind}}{m_c + 2m_r}', symbolic: '\\theta_{ss} = \\frac{F_{wind}}{F_e}', spoiler: true,
          note: 'The integrator on z finds the steady tilt that cancels the wind; without it z settles off target.' },
      ];
    },
    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch12;
      const ref = () => design(ctx.pModel, knobs(ctx.sys), 'sfi');
      const num = (v) => PD().num(v);
      PD().problemPanel(parent, ctx, prob, [
        {
          id: 'a', title: `(a) Gains with the F.8 pole pairs plus p<sub>I,h</sub> = ${prob.pIh}, p<sub>I,z</sub> = ${prob.pIz}`,
          html: 'Use the F.11 reference poles (the F.8 pairs: altitude t<sub>r</sub> = 8 s; lateral t<sub>r,z</sub> = 8 s and t<sub>r,θ</sub> = 0.8 s; ζ = 0.707) plus the integrator poles. Book sign: u = −Kx − k<sub>I</sub>x<sub>I</sub> with x<sub>I</sub> = ∫(r − y).',
          inputs: { Kh1: 'K<sub>h,1</sub>', Kh2: 'K<sub>h,2</sub>', kIh: 'k<sub>I,h</sub>', Kz1: 'K<sub>z,1</sub>', Kz2: 'K<sub>z,2</sub>', Kz3: 'K<sub>z,3</sub>', Kz4: 'K<sub>z,4</sub>', kIz: 'k<sub>I,z</sub>' },
          check: (v) => { const r = ref(); return PD().checkNumbers(v, { Kh1: r.Kh[0], Kh2: r.Kh[1], kIh: r.kIh, Kz1: r.Kz[0], Kz2: r.Kz[1], Kz3: r.Kz[2], Kz4: r.Kz[3], kIz: r.kIz }, { kIh: 'kI,h', kIz: 'kI,z' }); },
          solution: () => { const r = ref(); return [{ tex: `K_h = ${texMat([r.Kh])},\\; k_{I,h} = ${tex(r.kIh)},\\quad K_z = ${texMat([r.Kz])},\\; k_{I,z} = ${tex(r.kIz)}` }, { html: '[K k<sub>I</sub>] = place(A<sub>1</sub>, B<sub>1</sub>, p) for each loop, with the augmented A<sub>1</sub>, B<sub>1</sub> of Eq. 12.1 (p. 198).' }]; },
        },
        WB.myCtrl.part(ctx, {
          id: 'a2', title: '(a) Add integrators with anti-windup to your F.11 controller', seed: 'F.11/e',
          html: `Use the gains from above. The check (1) runs 1 m steps in h<sub>r</sub> and z<sub>r</sub> for 20 s with the nominal and with other parameters and compares h(t) and z(t) with the design (within 3 cm), then (2) lowers f<sub>max</sub> to ${WINDUP.fmax} N (just above hover) and steps h<sub>r</sub> to ${WINDUP.step} m, so a rotor saturates for seconds: h may overshoot by at most ${WINDUP.os} m.`,
          check: async (code) => {
            const cases = WB.myCtrl.paramCases(ctx).map((pc) => {
              const sc = F.scenario(ctx, { params: pc.params, refs: [STEP(1), STEP(1)], tEnd: 20, feed: 'state' });
              return { sc, label: pc.label, ref: () => refSS(ctx, sc, 'sfi') };
            });
            const m = await F.matchAll(ctx, code, cases, { tol: { 0: 0.03, 1: 0.03 } });
            if (!m.ok) return m;
            const sc = F.scenario(ctx, { params: { ...ctx.pModel, f_max: WINDUP.fmax }, refs: [STEP(WINDUP.step), STEP(0)], tEnd: WINDUP.tEnd, feed: 'state' });
            const res = await WB.myCtrl.run(ctx, code, sc);
            if (res.ok === false) return res;
            const os = Math.max(...res.yAll[1]) - WINDUP.step;
            if (!(os <= WINDUP.os)) return { ok: false, msg: `Tracking matches, but with f_max = ${WINDUP.fmax} N the ${WINDUP.step} m climb overshoots by ${fm(os)}: the integrators wind up while a rotor is saturated.` };
            return { ok: true, msg: `${m.msg} Saturated ${WINDUP.step} m climb: overshoot ${fm(Math.max(0, os))}.` };
          },
          solution: () => [
            { code: SOL.f12 },
            { html: `Anti-windup here holds both integrators while a rotor saturates (f<sub>max</sub> = ${WINDUP.fmax} N: about 0.23 m overshoot, versus 1.4 m without anti-windup). Integrating only while the loop is nearly settled, or unwinding by the saturation error (back-calculation), pass too.` },
          ],
        }),
        {
          id: 'b', title: '(b) 20% parameter variation and a 0.1 N wind force',
          html: 'In your dynamics, add F<sub>wind</sub> to the z equation (the hint in the statement) and let the parameters vary. Here: the wind F<sub>wind</sub> slider and the plant mismatch in the left panel (the chapter starts with both). Passes when both are set and every mismatch is within ±20%.',
          check: () => {
            const mis = ctx.S.mismatch || {}, v = Object.values(mis).map((x) => x || 0);
            const w = F.distValues(ctx).Fwind || 0;
            if (!w) return { ok: false, msg: 'Set the wind force F_wind (left panel, disturbances).' };
            if (!v.some((x) => Math.abs(x) > 0)) return { ok: false, msg: 'Set a plant mismatch in the left panel.' };
            return v.every((x) => Math.abs(x) <= 20.0001) ? { ok: true, msg: `F_wind = ${fmt(w, 3)} N with plant mismatch.` } : { ok: false, msg: 'Keep every parameter within ±20%.' };
          },
        },
        WB.myCtrl.part(ctx, {
          id: 'c', title: '(c) Tune the integrator poles (and other gains if needed) for good tracking', seed: ['F.12/a2'],
          html: `The check runs a 2 m altitude step and a z<sub>r</sub> step to 5.5 m for 50 s, with F<sub>wind</sub> = ${prob.dists.Fwind} N and the plant off by ${misText(ctx, prob.mismatch)}: |h<sub>r</sub> − h| and |z<sub>r</sub> − z| at the end must both be under 2 cm.`,
          check: async (code) => {
            const sc = F.scenario(ctx, { refs: [STEP(2), STEP(2.5)], zOff: 3, tEnd: 50, mismatch: prob.mismatch, ext: { Fwind: prob.dists.Fwind }, feed: 'state' });
            const res = await WB.myCtrl.run(ctx, code, sc);
            if (res.ok === false) return res;
            const eh = Math.abs(F.endErr(res, 1)), ez = Math.abs(F.endErr(res, 0));
            return { ok: eh < 0.02 && ez < 0.02, msg: `|h_r − h| = ${fm(eh)}, |z_r − z| = ${fm(ez)} at t = 50 s.` };
          },
          solution: () => [{ code: SOL.f12 }, { html: `The design from (a) (p<sub>I,h</sub> = ${prob.pIh}, p<sub>I,z</sub> = ${prob.pIz}) already passes: both integrators settle the loops despite the wind and the parameter errors. Faster integrator poles speed up the recovery but add overshoot.` }],
        }),
      ]);
    },
  });

  // ------------------------------------------------------------ Chapter 13 --
  function estCards(ctx, level) {
    const S = sub(ctx.pModel), d = design(ctx.pModel, ctx.st.k, level);
    const Ol = L.obsv(S.lon.A, S.lon.C), Oz = L.obsv(S.lat.A, S.lat.C);
    return [
      { title: 'Observability', page: 'p. 221',
        theory: '\\mathcal{O}_{A,C} = \\begin{bmatrix}C\\\\ CA\\\\ \\vdots\\\\ CA^{n-1}\\end{bmatrix},\\quad \\text{observable} \\iff \\operatorname{rank}\\mathcal{O}_{A,C} = n' },
      { title: 'Observability of the VTOL', page: 'F.13(b) p. 400', answers: 'F.13/b',
        numbers: `\\operatorname{rank}\\mathcal{O}_{lon} = ${L.rank(Ol)},\\quad \\operatorname{rank}\\mathcal{O}_{lat} = ${L.rank(Oz)}\\;(C_{lat} \\text{ is } 2\\times4)` },
      // the workbench's own choice of L (one of many): Explore only
      ...(ctx.S.mode === 'explore' ? [{ title: 'Lateral observer gain (workbench block structure)', page: 'p. 222 · Eq. 13.16',
        theory: 'L_{lat} = \\begin{bmatrix}L_{z1} & 0\\\\ 0 & L_{\\theta1}\\\\ L_{z2} & 0\\\\ 0 & L_{\\theta2}\\end{bmatrix}',
        note: 'With two outputs L is not unique. This structure lets the z innovation correct only (ẑ, ż̂) and the θ innovation only (θ̂, θ̇̂).' }] : []),
      { title: 'Observer gains for the VTOL', page: 'F.13(c) p. 400', answers: ['F.6/b', 'F.13/c'],
        symbolic:'\\operatorname{eig}(A - LC) = \\operatorname{eig}\\begin{bmatrix}-L_{z1} & 1\\\\ -L_{z2} & -\\frac{\\mu}{M}\\end{bmatrix} \\cup \\operatorname{eig}\\begin{bmatrix}-L_{\\theta1} & 1\\\\ -L_{\\theta2} & 0\\end{bmatrix}',
        numbers: d.Lz ? `L_h = ${texMat(d.Lh)},\\quad (L_{z1}, L_{z2}) = (${tex(d.Lz[0])}, ${tex(d.Lz[1])}),\\quad (L_{\\theta1}, L_{\\theta2}) = (${tex(d.Lt[0])}, ${tex(d.Lt[1])})` : '',
        note: 'place() on the full (A, C) would return a different L with the same eigenvalues.' },
      { title: 'Observer', page: 'p. 216 · Eq. 13.3, p. 224', answers: 'F.13/c',
        theory: '\\dot{\\hat x} = A\\hat x + B\\tilde u + L(y - C\\hat x),\\quad \\tilde u = (F_{sat} - F_e,\\; \\tau_{sat})' },
    ];
  }
  function obsExtraPlot(ctx, res) {
    if (ctx.S.mode === 'work') return workExtraPlot(ctx, res);
    if (ctx.st.extra === 'd') {
      const dv = F.distValues(ctx), S = ctx.S, on = (t) => t >= S.sim.tDist;
      return { opts: { title: 'disturbance estimates', yLabel: 'd̂', unit: '' }, data: { series: [
        { label: 'd̂_F [N]', y: Array.from(res.extras.dFhat || []), color: '--series-1' },
        { label: 'd̂_z [N]', y: Array.from(res.extras.dzhat || []), color: '--series-3' },
        { label: 'd̂_τ [N·m]', y: Array.from(res.extras.dthat || []), color: '--series-2' },
        // what d̂_F should converge to: d_F + M_true d_h plus the weight error (M − M_true) g of F_e.
        // The weight error would show F_e (F.4(a)), so Work mode adds it once that is solved.
        F.feShown(ctx)
          ? { label: 'equivalent altitude d (incl. weight error)', y: Array.from(res.t, (t) => (on(t) ? (dv.dF || 0) + WB.systems.F.mass(ctx.pTrue) * (dv.ah || 0) : 0) + (WB.systems.F.mass(ctx.pModel) - WB.systems.F.mass(ctx.pTrue)) * ctx.pModel.g), color: '--text-muted', dash: [5, 4], width: 1.5 }
          : { label: 'external altitude d (d_F + M d_h)', y: Array.from(res.t, (t) => (on(t) ? (dv.dF || 0) + WB.systems.F.mass(ctx.pTrue) * (dv.ah || 0) : 0)), color: '--text-muted', dash: [5, 4], width: 1.5 },
      ] } };
    }
    return { opts: { title: 'velocities and estimates', yLabel: 'velocity [m/s]', unit: 'm/s' }, data: { series: [
      { label: 'ż̂', y: Array.from(res.extras.zdhat || []), color: '--series-3', dash: [3, 3], width: 2 },
      { label: 'ż', y: res.x.map((x) => x[3]), color: '--series-1' },
      { label: 'ḣ̂', y: Array.from(res.extras.hdhat || []), color: '--series-2', dash: [3, 3], width: 2 },
      { label: 'ḣ', y: res.x.map((x) => x[4]), color: '--text-muted' },
    ] } };
  }

  // Work mode: the true velocities with the student's x_hat (ż̂ = x_hat[3], ḣ̂ = x_hat[4]),
  // or the d_hat they return (in their own order).
  function workExtraPlot(ctx, res) {
    const X = res.extras;
    if (ctx.st.extra === 'd') {
      const ds = ['dhat', 'dhat1', 'dhat2', 'dhat3'].filter((k) => X[k]);
      return { opts: { title: 'your disturbance estimates', yLabel: 'd̂', unit: '' }, data: { series: ds.length
        ? ds.map((k, i) => ({ label: `your d̂[${i}]`, y: Array.from(X[k]), color: ['--series-1', '--series-3', '--series-2', '--text-muted'][i] }))
        : [{ label: 'return (u, x_hat, d_hat) to plot d̂', y: Array.from(res.t, () => 0), color: '--text-muted', dash: [3, 3] }] } };
    }
    const series = [
      { label: 'ż', y: res.x.map((x) => x[3]), color: '--series-1' },
      { label: 'ḣ', y: res.x.map((x) => x[4]), color: '--text-muted' },
    ];
    if (X.xhat3) series.unshift({ label: 'your ż̂', y: Array.from(X.xhat3), color: '--series-3', dash: [3, 3], width: 2 });
    if (X.xhat4) series.push({ label: 'your ḣ̂', y: Array.from(X.xhat4), color: '--series-2', dash: [3, 3], width: 2 });
    return { opts: { title: 'velocities and your estimates', yLabel: 'velocity [m/s]', unit: 'm/s' }, data: { series } };
  }

  base('obs', 13, 'Observers', 'pp. 215–238, F.13 p. 400', {
    defaults(sys) { return { view: 'lat', zOff: 3, hOff: 0, antiwindup: 'clamp', extra: 'v', k: knobs(sys), w: slowW(sys, 'obs') }; },
    simDefaults(sys) { return { ...sys.problems.ch13.sim, refs: [{ type: 'square', amplitude: 2.5, frequency: 0.04 }] }; },
    buildControls(parent, ctx) {
      if (ctx.S.mode === 'work') { workControls(parent, ctx, 'F.13(c)'); return; }
      const sec = section(parent, 'Controller uses x̂', 'p. 222 · Fig. 13-3');
      viewSeg(sec, ctx);
      segmented(sec, { label: 'Extra plot', options: [{ value: 'v', label: 'velocities' }, { value: 'd', label: 'disturbances' }], ...bind(ctx, 'extra') });
      knobControls(parent, ctx, 'obs', 'Design'); gainReadout(section(parent, 'Gains', 'p. 222'), ctx, 'obs');
    },
    extraPlot: obsExtraPlot,
    math(ctx) { return [...estCards(ctx, 'obs'), { title: 'Separation principle', page: 'p. 223', theory: '\\operatorname{eig} = \\operatorname{eig}(A - BK) \\cup \\operatorname{eig}(A - LC)', note: 'For the linear model. Saturation and the nonlinear plant break it.' }]; },
    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch13;
      const S = () => sub(ctx.pModel);
      PD().problemPanel(parent, ctx, prob, [
        {
          id: 'a', title: '(a) Exact parameters, no input disturbance',
          html: 'Use α = 0 in your dynamics. Here: set every plant mismatch and every disturbance in the left panel to zero (the chapter starts that way). Passes when they are all zero.',
          check: () => {
            const mis = Object.values(ctx.S.mismatch || {}).some((x) => Math.abs(x || 0) > 0);
            const dist = Object.values(F.distValues(ctx)).some((x) => Math.abs(x || 0) > 0);
            if (mis) return { ok: false, msg: 'Set the plant mismatch to zero (α = 0).' };
            return dist ? { ok: false, msg: 'Set the disturbances to zero.' } : { ok: true, msg: 'The controller knows the true parameters, and nothing disturbs the plant.' };
          },
        },
        {
          id: 'b', title: '(b) Observability ranks', inputs: { rl: 'rank 𝒪 (altitude)', rz: 'rank 𝒪 (lateral)' },
          check: (v) => PD().checkNumbers(v, { rl: L.rank(L.obsv(S().lon.A, S().lon.C)), rz: L.rank(L.obsv(S().lat.A, S().lat.C)) }, {}),
          solution: () => [{ tex: `\\mathcal{O}_{lat} = ${texMat(L.obsv(S().lat.A, S().lat.C), 3)}` }, { html: 'Ranks 2 and 4. The lateral pair is observable from z and θ together (and even from z alone).' }],
        },
        WB.myCtrl.part(ctx, {
          id: 'c', title: '(c) Add an observer and use x̂ in your F.12 controller; tune the controller and observer poles', seed: ['F.12/c', 'F.12/a2'],
          html: `Your controller now gets only y = [[z], [h], [θ]]. Return <code>(u, x_hat)</code> from <code>update</code>, with x_hat = [z, h, θ, ż, ḣ, θ̇] (your estimate of the whole state). The check starts the VTOL at z = h = 0.5 m (your estimate starts wherever you start it) and runs a 2 m altitude step and a z<sub>r</sub> step to 5.5 m for 30 s with exact parameters: over the last 10 s, |z − ẑ| and |h − ĥ| must stay under 1 mm, |θ − θ̂| under 0.01°, |ż − ż̂| and |ḣ − ḣ̂| under 5 mm/s and |θ̇ − θ̇̂| under 0.1°/s, and |h<sub>r</sub> − h|, |z<sub>r</sub> − z| at the end under 3 cm.`,
          check: async (code) => {
            const sc = F.scenario(ctx, { refs: [STEP(2), STEP(2.5)], zOff: 3, init: { z0: 0.5, h0: 0.5 }, tEnd: 30 });
            const res = await WB.myCtrl.run(ctx, code, sc);
            if (res.ok === false) return res;
            const nx = F.needXhat(res);
            if (nx) return nx;
            const e = F.estErr(res, [[20, 30]]), D = M.DEG;
            const lim = [1e-3, 1e-3, 0.01 * D, 5e-3, 5e-3, 0.1 * D];
            const eh = Math.abs(F.endErr(res, 1)), ez = Math.abs(F.endErr(res, 0));
            const msg = `Over the last 10 s: |z − ẑ| ≤ ${fm(e[0])}, |h − ĥ| ≤ ${fm(e[1])}, |θ − θ̂| ≤ ${fmt(e[2] / D, 3)}°, |ż − ż̂| ≤ ${fmt(e[3], 3)} m/s, |ḣ − ḣ̂| ≤ ${fmt(e[4], 3)} m/s, |θ̇ − θ̇̂| ≤ ${fmt(e[5] / D, 3)}°/s; at the end |h_r − h| = ${fm(eh)}, |z_r − z| = ${fm(ez)}.`;
            return { ok: e.every((v, i) => v < lim[i]) && eh < 0.03 && ez < 0.03, msg };
          },
          solution: () => [
            { code: SOL.f13 },
            { html: 'The F.12 controller on x̂, with observers 10× faster than each controller pair, driven by the previous saturated input (F̃ = f<sub>r</sub> + f<sub>ℓ</sub> − F<sub>e</sub>, τ = d(f<sub>r</sub> − f<sub>ℓ</sub>)) and integrated with RK4, as the repo\'s ctrlObserver.py does. With two lateral outputs the 4×2 L is not unique; place (scipy\'s algorithm, as python-control uses) picks a well-conditioned one, and any L that puts the observer poles where you want them passes, since the check looks at the estimates.' },
          ],
        }),
        {
          id: 'd', title: '(d) Plot the states and their estimates',
          html: 'Run your controller: the z, h and θ plots show your ẑ, ĥ, θ̂ (dashed) with the true outputs, and the extra plot your ż̂, ḣ̂ with the true velocities.',
        },
        {
          id: 'e', title: '(e) Add d<sub>F</sub> = 1.0 N and d<sub>τ</sub> = 0.1 N·m',
          html: 'Runs your controller from (c) with an input force disturbance of 1.0 N and an input torque disturbance of 0.1 N·m (exact parameters, the (c) references). Your observer has no model of them, so x̂ is biased, and integrators acting on x̂ leave the outputs off their references. Passes when the run shows the bias (Chapter 14 removes it). <em>Set them</em> puts the same disturbances in the left panel.',
          actions: [{ label: 'Set them', run: () => { ctx.S.sim.dist = 1.0; ctx.S.sim.dists[0] = 0.1; ctx.update(); return null; } }],
          check: async () => {
            const code = WB.myCtrl.savedCode(ctx, 'F.13/c');
            if (!code) return { ok: false, msg: 'Write your controller in (c) first.' };
            const sc = F.scenario(ctx, { refs: [STEP(2), STEP(2.5)], zOff: 3, tEnd: 30, ext: { dF: 1.0, dtau: 0.1 } });
            const res = await WB.myCtrl.run(ctx, code, sc);
            if (res.ok === false) return res;
            const nx = F.needXhat(res);
            if (nx) return nx;
            const b = [0, 1, 2].map((i) => F.bias(res, i, 25, 30));
            const pk = Math.max(...res.x.map((x) => Math.abs(x[2]))) / M.DEG;
            let msg = `With d_F = 1 N and d_τ = 0.1 N·m, over the last 5 s: z − ẑ = ${fm(b[0])}, h − ĥ = ${fm(b[1])}, θ − θ̂ = ${fmt(b[2] / M.DEG, 3)}°; at the end h_r − h = ${fm(-F.endErr(res, 1))}, z_r − z = ${fm(-F.endErr(res, 0))}.`;
            if (pk > 90) msg += ` Your VTOL loses control (|θ| reaches ${fmt(pk, 3)}°).`;
            if (Math.abs(b[0]) < 1e-3 && Math.abs(b[1]) < 1e-3) return { ok: false, msg: `${msg} x̂ follows the outputs exactly: is it coming from an observer of the model?` };
            return { ok: true, msg };
          },
        },
      ]);
    },
  });

  // ------------------------------------------------------------ Chapter 14 --
  base('dobs', 14, 'Disturbance observers', 'pp. 239–259, F.14 p. 401', {
    defaults(sys) { return { view: 'lat', zOff: 3, hOff: 0, antiwindup: 'clamp', dobs: true, extra: 'd', k: knobs(sys), w: slowW(sys, 'dobs') }; },
    simDefaults(sys) { const pr = sys.problems.ch14; return { ...pr.sim, refs: [{ type: 'square', amplitude: 2.5, frequency: 0.04 }], dists: [0, 0, pr.dists.wind, pr.dists.ah], mismatch: pr.mismatch }; },
    buildControls(parent, ctx) {
      const extra = [{ value: 'd', label: 'disturbance estimates' }, { value: 'v', label: 'velocities' }];
      if (ctx.S.mode === 'work') { workControls(parent, ctx, 'F.14(b)', extra); return; }
      const sec = section(parent, 'Controller uses x̂, subtracts d̂', 'p. 241');
      viewSeg(sec, ctx);
      segmented(sec, { label: 'Disturbance observer', options: [{ value: true, label: 'on' }, { value: false, label: 'off (F.14a)' }], ...bind(ctx, 'dobs') });
      segmented(sec, { label: 'Extra plot', options: extra, ...bind(ctx, 'extra') });
      knobControls(parent, ctx, 'dobs', 'Design'); gainReadout(section(parent, 'Gains', 'p. 241'), ctx, 'dobs');
    },
    extraPlot: obsExtraPlot,
    math(ctx) {
      const d = design(ctx.pModel, ctx.st.k, 'dobs');
      return [
        { title: 'Disturbance models', page: 'p. 240–241', answers: 'F.14/b',
          theory: '\\text{altitude: } \\dot{\\hat d}_F = L_{d,h}(h - \\hat h)\\text{ at the input};\\quad \\text{lateral: } d_\\tau \\text{ at the input, } d_z \\text{ a force in } \\ddot z',
          note: 'A constant wind speed w added to ż (the F.14 snippet) is exactly a force μw in the observer\'s ż coordinates, so the d_z state absorbs it.' },
        { title: 'Augmented blocks', page: 'p. 240',
          theory: 'A_2 = \\begin{bmatrix}A & B_d\\\\ 0 & 0\\end{bmatrix},\\quad C_2 = \\begin{bmatrix}C & 0\\end{bmatrix}\\;(\\dot d = 0)' },
        { title: 'Augmented VTOL blocks', page: 'F.14(b) p. 401', answers: ['F.6/a', 'F.6/b', 'F.14/b'],
          symbolic:'A_{h} = \\begin{bmatrix}0&1&0\\\\0&0&\\frac1M\\\\0&0&0\\end{bmatrix},\\; A_{z} = \\begin{bmatrix}0&1&0\\\\0&-\\frac{\\mu}{M}&\\frac1M\\\\0&0&0\\end{bmatrix},\\; A_{\\theta} = \\begin{bmatrix}0&1&0\\\\0&0&\\frac1J\\\\0&0&0\\end{bmatrix},\\; C = \\begin{bmatrix}1&0&0\\end{bmatrix}' },
        { title: 'Observer gains', page: 'p. 241',
          theory: 'L = \\text{place}(A^\\top, C^\\top, q)^\\top \\text{ per block}' },
        { title: 'Observer gains for the specs', page: 'F.14(b) p. 401', answers: 'F.14/b',
          numbers: `L_h = ${texMat(d.Lh)},\\; L_z = ${texMat(d.Lz)},\\; L_\\theta = ${texMat(d.Lt)}` },
        { title: 'Control law', page: 'p. 241', answers: 'F.14/b',
          theory: '\\tilde F = -K_h\\hat x_{lon} - k_{I,h}x_{I,h} - \\hat d_F,\\quad \\tau = -K_z\\hat x_{lat} - k_{I,z}x_{I,z} - \\hat d_\\tau',
          note: 'd_z is not matched to τ, so it is not cancelled directly; removing the estimator bias lets the z integrator do the rest.' },
      ];
    },
    buildProblem(parent, ctx) {
      const prob = ctx.sys.problems.ch14;
      const mis = misText(ctx, prob.mismatch);
      PD().problemPanel(parent, ctx, prob, [
        {
          id: 'a', title: '(a) α = 0.2 with altitude and wind disturbances',
          html: `The chapter starts with the plant off by ${mis}, a wind w = ${prob.dists.wind} m/s added to ż and an altitude disturbance d<sub>h</sub> = ${prob.dists.ah} m/s² added to ḧ (the book's snippet), in the left panel. Run your F.13 controller here to see how the estimates and the tracking suffer without a disturbance observer.`,
          actions: [{ label: 'Run my F.13 controller', run: () => {
            const code = WB.myCtrl.savedCode(ctx, 'F.13/c');
            if (!code) return { ok: false, msg: 'Write your controller in F.13(c) first (Ch 13 tab).' };
            return WB.myCtrl.use(ctx, code, 'a');
          } }],
        },
        WB.myCtrl.part(ctx, {
          id: 'b', title: '(b) Add a disturbance observer to both controllers; verify the estimator\'s steady-state error is removed, and tune', seed: ['F.13/c'],
          html: `Return <code>(u, x_hat)</code>, or <code>(u, x_hat, d_hat)</code> to plot your d̂ (extra plot). The check runs a 2 m altitude step and a z<sub>r</sub> step to 5.5 m for 30 s with the plant off by ${mis}, w = ${prob.dists.wind} m/s and d<sub>h</sub> = ${prob.dists.ah} m/s²: over the last 5 s the means of z − ẑ and h − ĥ must be under 2 mm, and |h<sub>r</sub> − h|, |z<sub>r</sub> − z| at the end under 3 cm.`,
          check: async (code) => {
            const sc = F.scenario(ctx, { refs: [STEP(2), STEP(2.5)], zOff: 3, tEnd: 30, mismatch: prob.mismatch, ext: { wind: prob.dists.wind, ah: prob.dists.ah } });
            const res = await WB.myCtrl.run(ctx, code, sc);
            if (res.ok === false) return res;
            const nx = F.needXhat(res);
            if (nx) return nx;
            const bz = F.bias(res, 0, 25, 30), bh = F.bias(res, 1, 25, 30);
            const eh = Math.abs(F.endErr(res, 1)), ez = Math.abs(F.endErr(res, 0));
            return { ok: Math.abs(bz) < 2e-3 && Math.abs(bh) < 2e-3 && eh < 0.03 && ez < 0.03, msg: `Mean over the last 5 s: z − ẑ = ${fm(bz)}, h − ĥ = ${fm(bh)}; at the end |h_r − h| = ${fm(eh)}, |z_r − z| = ${fm(ez)}.` };
          },
          solution: () => [
            { code: SOL.f14 },
            { html: `The F.13 observers augmented with constant disturbance states (p. 240–241), each pair 10× faster than its controller pair plus a disturbance pole (p<sub>d,h</sub> = ${prob.pDh}, p<sub>d,z</sub> = ${prob.pDz}, p<sub>d,θ</sub> = ${prob.pDth}): d<sub>F</sub> at the force input and d<sub>τ</sub> at the torque input are subtracted from F and τ; d<sub>z</sub>, a force in z̈, is not matched to τ, but removing the estimator bias lets the z integrator cancel it. With these disturbances d̂<sub>F</sub> settles at M<sub>true</sub>·d<sub>h</sub> plus the weight error (M − M<sub>true</sub>)g, and d̂<sub>z</sub> near μ<sub>true</sub>·w: a constant wind speed added to ż is exactly a force μw in the observer's coordinates.` },
          ],
        }),
      ]);
    },
  });

  WB.F.ss = { design, makeSS, sub, latL, knobs };
})();
