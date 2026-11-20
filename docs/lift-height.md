# Lift height and stay-down moves

For **Pocket** and **3D Adaptive Clearing**, open **Lead-in and lead-out moves →
Linking moves → Lift height (mm)**. The value is a distance above each cutting
level. 3D Adaptive presets **0.2 mm** and uses that as the minimum for links
over a verified cleared floor.

For example, a 2 mm lift connects passes cutting at Z −5 mm at Z −3 mm, then
returns to Z −5 mm before the next cut. This helps keep the tool off the floor
during transitions and reduces witness marks.

Local concentric ring links in **Pocket** can lift and traverse at cutting feed,
then use the selected plunge, ramp or helix to return to depth. In **3D Adaptive**,
joins that cut into stock stay at cutting depth. Verified floor links rise along
the lead-out arc, traverse at lift height, then descend along the lead-in arc.
Set their radii independently under **Leads and transitions**. Both default to
**0.6 mm**. A zero radius disables that arc. When an arc cannot fit in the cleared
area, the link can lift or descend vertically at a verified point.

Lifts stop at the operation's clearance height. Disconnected regions and distant
links still retract fully to clearance, and 3D Adaptive still checks the swept
cutter against protected model geometry. Lift height must be finite and zero or
positive. It is saved with the operation and supports Apply, Cancel and undo.

**3D Parallel** automatically connects eligible rows with surface cutting moves.
Its **Retraction policy** controls repositioning between cutting chains.
See [Parallel finishing](parallel.md).

Other strategies do not expose this control. Offset Roughing and Horizontal are
not separate strategies in the app.
