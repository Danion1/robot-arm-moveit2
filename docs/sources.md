# Sources

Every number used in the CAD and the calculations comes from one of these. Manufacturer files (STEP, PDF) are **not redistributed** here: download them from the links below.

## Servos (ROBOTIS)

| What | Where | Used for |
|---|---|---|
| XM430-W350-T e-Manual | https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/ | dimensions, mass, stall torque, speed, operating modes, overload shutdown (`cad/lib/dados_servos_dynamixel.py`) |
| XC430-W240-T e-Manual | https://emanual.robotis.com/docs/en/dxl/x/xc430-w240/ | same (the XC430 has no current/torque mode) |
| X430 drawing and 3D model, "Drawings → X430" on the XM430 page | https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#drawings | hole pattern (`X430_FURACAO`), body shape, servo solid for collision checks |
| XM430/XH430 moment of inertia (PDF, same "Drawings" section) | same page | servo inertia in the URDF (`cad/massa.py`) |
| FR12-H101K and FR12-S101K brackets (same "Drawings" section) | same page | reference only: plate thickness and screw engagement used by ROBOTIS |

### The servo STEP file (needed to run the CAD scripts)

The scripts load the official X430 model and check its SHA-256 before using it (`cad/servo_step.py`), so the CAD is always checked against the same revision.

1. Download the X430 idler STEP from the "Drawings" section above (listed on the site as `x-430_idle.stp`; it downloads as `XM,H-430_idler.stp`). A browser is needed: direct download with `curl` returns the home page.
2. Save it as `fontes/fabricante/XM,H-430_idler.stp` (this folder is ignored by Git).
3. Expected SHA-256: `7ff4e39475245d5c1fc4f703e9241fca1a09d57aed920274498dbe2cd5e31e22`. If it differs, the scripts stop with a clear message.

`fontes/medir_x430_step.py` cross-checks the hole table typed from the PDF drawing against the STEP: 34 threaded holes, 4 housing-screw counterbores, the faces and the body box, all within 0.01 mm (a deliberate 1 mm error makes it fail).

## Material

| What | Where | Used for |
|---|---|---|
| PLA Technical Data Sheet, SD3D (copy hosted by the Harvard 3D printing lab) | https://3dp.seas.harvard.edu/files/2023/01/PLAMDS.pdf | density 1.24 g/cm³, tensile modulus 2.3 GPa, yield 35.9 MPa, HDT 49 °C (`cad/lib/dados_materiais.py`). Values are for solid material; Poisson's ratio 0.35 is ASSUMED (not in the sheet). |

## Fasteners

| Standard | Where | Used for |
|---|---|---|
| ISO 4762 socket head cap screw | https://fasten.it/norms/norm/din_en_iso_4762 | head diameter and height (`cad/lib/parafusos_iso.py`) |
| ISO 273 clearance holes | https://optimas.com/technical-resources/tapping-sizes/ | through holes |
| ISO 4032 hex nut | https://fasten.it/norms/norm/din_en_iso_4032 | embedded nut pockets |
| Counterbores for ISO 4762 | https://engineersbible.com/counterbore-socket-iso/ | screw head recesses |
| ISO 2306 tap drill | — | Ø1.6 for M2, Ø2.05 for M2.5 in the servo hole check |

## Gears

| What | Where | Used for |
|---|---|---|
| Lewis form factor Y, 20° full-depth teeth | Shigley, *Mechanical Engineering Design*, Table 14-2 | pinion bending check (`cad/engrenagem.py`). A second source for Y is still pending. |
