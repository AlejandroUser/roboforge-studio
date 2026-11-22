---
name: mechanics-math
description: >-
  Fórmulas de mecánica estructural para brazos y eslabones robóticos ahusados:
  flexión, momento de inercia efectivo por arquitectura, deflexión en la punta,
  inercia rotacional y torque dinámico.
---

# Skill: Mecánica Estructural para Eslabones Robóticos

## Fórmulas de Referencia (Unidades SI: mm, N, MPa, kg)

1. **Momento flector máximo en raíz (x = 0):**
   - $M_{\max} = F_{\text{tip}} \cdot L + \frac{1}{2} m_{\text{link}} \cdot g \cdot L$ (en `N·mm`, con $g = 9.81\,\text{m/s}^2$).
2. **Momento de inercia de sección en raíz ($I_{\text{root}}$ en $\text{mm}^4$):**
   - Sólido rectangular equivalente: $I_{\text{rect}} = \frac{b \cdot h^3}{12}$
   - Perfil en I (*I-Beam*): $I_{\text{I}} = \frac{b \cdot h^3 - (b - t_w)(h - 2 t_f)^3}{12}$
   - Carcasa hueca (*Hollow Shell*): $I_{\text{box}} = \frac{b \cdot h^3 - (b - 2 t)(h - 2 t)^3}{12}$
   - Celosía (*Truss*): modelada como dos cordones superior/inferior de espesor $t_c$ separados por $h - 2 t_c$, con factor de eficiencia por cortante de diagonales ($\eta \approx 0.82$).
3. **Esfuerzo máximo de flexión ($\sigma_{\max}$ en MPa):**
   - $\sigma_{\max} = K_t \cdot \frac{M_{\max} \cdot (h/2)}{I_{\text{root}}}$
   - Factor de concentración de esfuerzos $K_t$: `1.15` (ahusado suave con fillets), `1.25` (perfil I), `1.35` (celosía redondeada), `1.18` (carcasa hueca).
4. **Deflexión máxima en la punta ($\delta_{\max}$ en mm):**
   - Para viga ahusada, usar integración numérica de $\int_0^L \frac{M(x) \cdot x}{E \cdot I(x)} dx$ (vía regla de Simpson en 40 puntos con `numpy`) para precisión real en secciones variables!
5. **Inercia rotacional respecto a la raíz ($I_{\text{rot}}$ en $\text{kg}\cdot\text{cm}^2$):**
   - $I_{\text{rot}} \approx m_{\text{link}} \cdot (L_{\text{cm}})^2 \cdot c_{\text{dist}} + m_{\text{payload}} \cdot L_{\text{cm}}^2$, donde $c_{\text{dist}} \in [0.22, 0.33]$ refleja la distribución de masa ahusada.
6. **Torque dinámico requerido ($\tau_{\text{din}}$ en $\text{N}\cdot\text{m}$):**
   - $\tau_{\text{din}} = (I_{\text{rot, kg}\cdot\text{m}^2}) \cdot \alpha + (F_{\text{tip}} + 0.5 m_{\text{link}} g) \cdot L_{\text{m}}$
