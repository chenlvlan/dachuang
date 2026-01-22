/*
 * compute.c
 *
 *  Created on: Jan 22, 2026
 *      Author: yufei
 */

#include "compute.h"

static inline float clampf(float x, float min, float max) {
	if (x < min)
		return min;
	if (x > max)
		return max;
	return x;
}

//const static float legData[5] = { 90.0, 90.0, 130.0, 130.0, 65.5 };

IKStatus_t fivebar_inverse_kinematics(float x, float y, const FiveBarGeom_t *g,
		const FiveBarLimit_t *lim, float *theta_f, float *theta_r) {
	/* ================= 前腿 ================= */
	float rf2 = x * x + y * y;
	if (rf2 < EPS)
		return IK_NUMERIC_ERROR;

	float rf = sqrtf(rf2);

	if (rf > (g->L1 + g->L3) || rf < fabsf(g->L1 - g->L3))
		return IK_OUT_OF_REACH;

	float cos_af = (g->L1 * g->L1 + rf * rf - g->L3 * g->L3)
			/ (2.0f * g->L1 * rf);
	cos_af = clampf(cos_af, -1.0f, 1.0f);

	float alpha_f = acosf(cos_af);

	/* 数学角：+Y 为 0，逆时针为正 */
	float phi_f = atan2f(x, y);

	/* 选择膝盖朝前（凸结构） */
	float th_f = phi_f - alpha_f;

	/* ================= 后腿 ================= */
	/* 后电机在 (-d, 0) */
	float xr = x + g->d;
	float yr = y;

	float rr2 = xr * xr + yr * yr;
	if (rr2 < EPS)
		return IK_NUMERIC_ERROR;

	float rr = sqrtf(rr2);

	if (rr > (g->L2 + g->L4) || rr < fabsf(g->L2 - g->L4))
		return IK_OUT_OF_REACH;

	float cos_ar = (g->L2 * g->L2 + rr * rr - g->L4 * g->L4)
			/ (2.0f * g->L2 * rr);
	cos_ar = clampf(cos_ar, -1.0f, 1.0f);

	float alpha_r = acosf(cos_ar);

	float phi_r = atan2f(xr, yr);

	/* 选择膝盖朝后（凸结构） */
	float th_r = phi_r + alpha_r;

	th_f = th_f;
	th_r = 0.0f - th_r; //关节角取反

	/* ---------- 关节限位 ---------- */
	if (lim) {
		if (th_f < lim->theta_f_min || th_f > lim->theta_f_max
				|| th_r < lim->theta_r_min || th_r > lim->theta_r_max)
			return IK_JOINT_LIMIT;
	}

	*theta_f = th_f;
	*theta_r = th_r;

	return IK_OK;
}

