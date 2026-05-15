# SoftToss

### Spherical ball simulator with a custom physics system using C++

- Accurate drag, magnus effect, and gravitational forces act upon the ball.
- Ball flight considers the drag crisis, spin down, seam-orientation-dependant lift, and seam shifted wake.
- Bat to object contact utilizes an oblique collision model.
- Surface contact considers restitution and friction.
- x right
- y forward
- z upward

## Math

### Aerodynamics

> $$ p_{air} = p_{bar} e^{-\beta h} $$
> $$ SVP = 4.5841 e^{\frac{ (18.687 - \frac{T}{234.5}) T}{257.14 + T} } $$
> $$ \rho = 1.2929 \times \frac{273}{T + 273} \times \frac{p_{air} - 0.379 (SVP \times RH / 100)}{760} $$

---

Given: $ \vec{r}_0$, $\vec{v}_0$, $\vec{\omega}_0$, $m$, $r$, $t$, $ \rho$, $\|v_{wind}\|$, $phi_{wind}$, $h_{wind}$

Find: $ \vec{F}$, $\vec{a}$, $\vec{v}$, $\vec{r}$, $\tau$, $\vec{\alpha}$, $\vec{\omega}$

---

If $z \geq h_{wind}$:

> $$ v_{wind} = \langle \|v_{wind}\| \cos{(phi_{wind})},  \|v_{wind}\| \cos{(phi_{wind})}, 0 \rangle $$

Else:

> $$ v_{wind} = \langle 0,  0, 0 \rangle $$

> $$ v_{rel} = \sqrt{(v_x-v_{wind_x})^2 + (v_y-v_{wind_y})^2 + (v_y-v_{wind_y})^2} $$

> $$ c_{d_0} = 0.3008 $$
>
> $$ c_d = c_{d_0} + 0.0292 (\frac{\omega_{active}}{1000}) $$
>
> $$ s = \frac{r * \omega_{active}}{\|v_{rel}\|} $$
>
> $$ c_l = \frac{1.120 \times S}{0.583 + 2.333 \times S} $$

> $$ \vec{F}_d = \frac{1}{2} \rho A c_d \|v\| \vec{v} $$
>
> $$ \vec{F}_m = \frac{1}{2} \rho A c_l v_{rel}^2 (\frac{\omega \times v_{rel}}{\| \omega \times v_{rel} \|}) $$
>
> $$ \vec{F}_g = (mg)\hat{k} $$
>
> $$ \Sigma\vec{F} = F_d + F_m + F_g $$

> $$ \vec{a} = \vec{F} / m $$

> $$ \vec{v} = \vec{v}_0 + \vec{a} (t) $$

> $$ \vec{r} = \vec{r}_0 + \vec{v} (t) + \frac{1}{2} \vec{a} (t)^2 $$

> $$ k = 0.02 $$
>
> $$ \tau = k r \|\vec{F}_m\| $$
>
> $$ I = \frac{2}{5} m r^2 $$
>
> $$ \vec{\alpha} = -\frac{\tau}{I}\hat{\omega} $$

> $$ \vec{\omega} = \vec{\omega} + \vec{\alpha} (t) $$

### Collisions

#### Ball-Bat

Given:

$$ \vec{r}_{ball}, \vec{v}_{0_{ball}}, \vec{\omega}_{0_{ball}}, m_{ball}, r_{ball}, \vec{r}_{bat}, \vec{v}_{0_{bat}}, \vec{\omega}_{0_{bat}}, m_{bat}, r_{bat}, I_{ball}, I_0, I_z $$

Find:

$$ \vec{v}_{f_{ball}}, \vec{\omega}_{f_{ball}} $$

---

> $$ \vec{d} = \vec{r}_{bat} - \vec{r}_{bat,cm} $$

> $$ \hat{n} = \frac{ \vec{r}_{ball} - \vec{r}_{bat} }{ \| \vec{r}_{ball} - \vec{r}_{bat} \| } $$

> $$ \vec{v}_{c_{ball}} = \vec{v}_{0_{ball}} - r_{ball}( \vec{\omega}_{0_{ball}} \times \hat{n}) $$
>
> $$ \vec{v}_{c_{bat}} = \vec{v}_{0_{bat}} + ( \vec{\omega}_{0_{bat}} \times d) $$
>
> $$ \vec{v}_{rel} = \vec{v}_{c_{ball}} - \vec{v}_{c_{bat}} $$

> $$ \hat{t} = \frac{ \vec{v}_{rel} - (\vec{v}_{rel} \cdot \hat{n}) \hat{n} }{ \| \vec{v}_{rel} - (\vec{v}_{rel} \cdot \hat{n}) \hat{n} \| } $$
>
> $$ \hat{b} = \hat{n} \times \hat{t} $$

> $$ v_n = (\vec{v}_{rel}) \cdot (\hat{n}) $$
>
> $$ v_t = (\vec{v}_{rel}) \cdot (\hat{t}) $$
>
> $$ v_b = (\vec{v}_{rel}) \cdot (\hat{b}) $$

> $$ e_n = 0.61 - (0.001) \|v_{rel}\| (\frac{0.682 mph}{ft/s}) $$
>
> $$ e_t = 0.405 [baseballs] = 0.146 [softballs] $$

> $$ \frac{1}{m_{eff_n}} = \frac{1}{m_{ball}} + \frac{1}{m_{bat}} + \frac{\|\vec{d} \times \hat{n}\|^2}{I_0} $$
>
> $$ J_n = \frac{-(1 + e_n) v_n}{1/m_{eff_n}} $$

> $$ \frac{1}{m_{eff_t}} = \frac{1}{m_{ball}} + \frac{1}{m_{bat}} + \frac{\|\vec{d} \times \hat{t}\|^2}{I_0} + \frac{r_{ball}^2}{I_{ball}} $$
>
> $$ J_{t} = \frac{-(1 + e_t) v_t}{1/m_{eff_t}} $$

> $$ \frac{1}{m_{eff_b}} = \frac{1}{m_{ball}} + \frac{1}{m_{bat}} + \frac{\|\vec{d} \times \hat{b}\|^2}{I_z} + \frac{r_{ball}^2}{I_{ball}} $$
>
> $$ J_{b} = \frac{-(1 + e_t) v_b}{1/m_{eff_b}} $$

> $$ \vec{J} = (J_n)(\hat{n}) + (J_t)(\hat{t}) + (J_b)(\hat{b}) $$

> $$ \vec{v}_{f_{ball}} = \vec{v}_{0_{ball}} + \frac{\vec{J}}{m_{ball}} $$
>
> $$ \vec{\omega}_{f_{ball}} = \vec{\omega}_{0_{ball}} + \frac{-r_{ball} (\vec{J} \times \hat{n})}{I_{ball}} $$

#### Ball-Surface

Given:

$$ \vec{r}_{ball}, \vec{r}_{s}, \vec{v}_0, \vec{\omega}_0, m, r, I, e_n, \mu $$

Find:

$$ \vec{v}_f, \vec{\omega}_f $$

---

> $$ \hat{n} = \frac{ \vec{r}_{ball} - \vec{r}_{s} }{ \| \vec{r}_{ball} - \vec{r}_{s} \| } $$

> $$ \vec{J}_n = -m(1 + e_n) (v_0 \cdot \hat{n}) (\hat{n}) $$
>
> $$ J_{fmax} = \mu \|J_n\| $$

> $$ \vec{v}_{slip} = (\vec{v}_0 - (\vec{v}_0 \cdot \hat{n})\hat{n}) - r (\omega_0 \times \hat{n}) $$
>
> $$ \vec{J}_{slide} = -J_{fmax} \frac{v_{slip}}{\|v_{slip}\|} $$

> $$ \vec{J}_{grip} = -\frac{2}{7} m(1+e_t) v_{slip} $$

$$ If \|J_{grip}\| \le J_{fmax} $$
> $$ \vec{J}_t = \vec{J}_{grip} $$

$$ If \|J_{grip}\| > J_{fmax} $$
> $$ \vec{J}_t = \vec{J}_{slide} $$

> $$ \vec{J} = \vec{J}_n + \vec{J}_t $$

> $$ \vec{v}_f = \vec{v}_0 + \frac{\vec{J}}{m} $$
>
> $$ \vec{\omega}_f = \vec{\omega}_0 + \frac{(-r \hat{n}) \times \vec{J}}{I} $$

## References

<https://baseball.physics.illinois.edu/index.html>

<https://www.physics.usyd.edu.au/~cross/baseball.html>

<https://physics.csuchico.edu/baseball/>

<https://baseballaero.com/baseball-aerodynamics/>

<https://www.acs.psu.edu/drussell/bats.html>
