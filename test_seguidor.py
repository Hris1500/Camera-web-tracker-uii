"""Pruebas rápidas de la lógica sin cámara ni Arduino:  python -m pytest test_seguidor.py"""
from seguidor import PERSONA, Eje, asigna_caras, zona_mira


def test_eje_solo_actua_al_salir_y_vuelve_al_centro():
    e = Eje(10, 170, 60, signo=-1)
    e.actualiza(0.3, False, 90, borde=0.55, vuelta=0.12)
    assert e.cmd == 90 and not e.activo                 # dentro de la zona: quieto
    e.actualiza(0.8, False, 90, borde=0.55, vuelta=0.12)
    assert e.activo and 60 < e.cmd < 90                 # a la derecha -> pan baja (signo -1)
    e.actualiza(0.3, False, 80, borde=0.55, vuelta=0.12)
    assert e.activo                                     # histéresis: sigue corrigiendo
    e.actualiza(0.05, False, 75, borde=0.55, vuelta=0.12)
    assert not e.activo
    e.actualiza(1.0, True, 15, borde=0.55, vuelta=0.12)
    assert e.cmd == 10                                  # respeta el límite mecánico


def test_persona_apunta_a_su_cara_o_a_la_cabeza():
    p = dict(cls=PERSONA, box=(100, 100, 200, 400))
    asigna_caras([p], [(130, 110, 170, 150, 0.9), (130, 350, 170, 390, 0.95)])  # la 2ª está en las piernas
    assert zona_mira(p) == (130, 110, 170, 150)
    sin_cara = dict(cls=PERSONA, box=(100, 100, 200, 400))
    x1, y1, x2, y2 = zona_mira(sin_cara)
    assert y1 == 100 and y2 < 200 and x1 > 100 and x2 < 200
