'''
Written by yufeng.wu0902@gmail.com

Python class for FK/IK calculations on Q8bot.
'''

import math

# d - distance between motors; l1/l1p - upper linkage length; l2/l2p - lower linkage length
# Unit is in mm.
class k_solver:
    def __init__(self, d = 19.5, l1 = 25, l2 = 40, l1p = 25, l2p = 40):
        self.d = d
        self.l1 = l1
        self.l2 = l2
        self.l1p = l1p
        self.l2p = l2p
        self.prev_ik = [45, 135]

    # Solve inverse kinematics at an given end effector position
    def ik_solve(self, x, y, deg = True, rounding = 3):
        try:
            c1 = math.sqrt((x - self.d)**2 + y**2)
            c2 = math.sqrt(x**2 + y**2)
            a1 = math.acos((c1**2 + self.d**2 - c2**2) / (2*c1*self.d))
            a2 = math.acos((c2**2 + self.d**2 - c1**2) / (2*c2*self.d))
            b1 = math.acos((c1**2 + self.l1**2 - self.l2**2) / (2*c1*self.l1))
            b2 = math.acos((c2**2 + self.l1p**2 - self.l2p**2) / (2*c2*self.l1p))
            q1 = math.pi - a1 - b1
            q2 = a2 + b2
            if deg:
                q1, q2 = q1*180/math.pi, q2*180/math.pi
            self.prev_ik = [q1, q2]
            return round(q1, rounding), round(q2, rounding), True
        except:
            return self.prev_ik[0], self.prev_ik[1], False

    # Solve forward kinematics at an given joint angle pair.
    # 발끝은 두 원(중심 e1/e2, 반지름 l2/l2p)의 교점 - 표준 원-원 교점 공식(닫힌해).
    # 교점은 둘이나, fsolve(초기값 [10,60])가 수렴하던 쪽은 항상 xm + h*dy/D 부호
    # (y가 큰 쪽) - compare.py의 FK 샘플 16개로 확인.
    def fk_solve(self, q1, q2, deg = True, rounding = 3):
        if deg:
            angles = (self._deg2rad(q1), self._deg2rad(q2))
        q1, q2 = angles
        Xa = self.l1 * math.cos(q1) + self.d
        Ya = self.l1 * math.sin(q1)
        Xb = self.l1p * math.cos(q2)
        Yb = self.l1p * math.sin(q2)
        dx, dy = Xb - Xa, Yb - Ya
        D = math.hypot(dx, dy)
        if D == 0 or D > self.l2 + self.l2p or D < abs(self.l2 - self.l2p):
            raise ValueError("no intersection: circles do not meet")
        a = (self.l2**2 - self.l2p**2 + D**2) / (2 * D)
        h2 = self.l2**2 - a**2
        if h2 < 0:
            raise ValueError("no intersection: circles do not meet")
        h = math.sqrt(h2)
        xm, ym = Xa + a * dx / D, Ya + a * dy / D
        x, y = xm + h * dy / D, ym - h * dx / D
        return round(x, rounding), round(y, rounding)

    #-------------------#
    # Private Functions #
    #-------------------#
    def _deg2rad(self, ang_deg):
        return ang_deg*math.pi/180
