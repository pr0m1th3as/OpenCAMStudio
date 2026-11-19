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

**3D Parallel** exposes a separate **Keep tool down** checkbox in the same section.
When checked, **0 mm** Lift height continues finishing along the surface between
eligible passes, removing the intervening scallop without an added lift or
re-entry plunge. Transitions use the same model-contact and cutter checks as
the finishing rows. A positive value lifts above the higher endpoint
by the requested distance. The full cutter sweep is checked against all
setup bodies. Blocked links, gaps, split rows and long returns use full clearance;
lifts reaching clearance use the normal clearance rapid. Raised non-cutting
links are kept only where verification finds sufficient room in the remaining
stock. A blocked raised traverse triggers a clearance move and controlled
re-entry, followed by verification of the revised path. Zero-lift transitions
continue cutting the surface instead of treating the turn as a non-cutting traverse.
They traverse at cutting feed and, with a positive lift, descend at plunge feed. With **Keep tool
down** unchecked, Parallel uses full-clearance links regardless of Lift height.
The checkbox defaults to off in older projects. Parallel still requires
pre-roughed stock; see [Parallel finishing](parallel.md).

Other strategies do not expose this control. Offset Roughing and Horizontal are
not separate strategies in the app.
