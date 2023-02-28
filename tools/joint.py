#!/usr/bin/env python3
# SPDX-License-Identifier: CC0
import sympy as s

eqs = []
def add_eq(left, right):
    eqs.append(s.Eq(left, right))

def solve(var):
    ret = []
    for eq in eqs:
        if var in eq.free_symbols:
            ret.extend(s.solve(eq, var))
    return ret

a, Fg, Fm, I, Kt, Kw = s.symbols("a Fg Fm I Kt Kw")
m, Ng, r, R, Tm, Tp, v, V, Wm, Wp = s.symbols("m Ng r R Tm Tp v V Wm Wp")

add_eq(V, I*R + Kw*Wm)
add_eq(Tm, Kt*I)
add_eq(Ng*Tm, Tp)
add_eq(Fm*r, Tp)
add_eq(a*m, Fm + Fg)
add_eq(Wm, Ng*Wp)
add_eq(v, r*Wp)
