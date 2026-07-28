# Feline anatomy reference

The anatomical basis for T.O.M.C.A.T.'s biomimetic geometry (digitigrade legs,
fore/hind asymmetry, articulated spine, thoracic ribcage, tail). All sources
here are **public domain or openly licensed**, so they are safe to cite and
redistribute with this project.

## Primary source (public domain)

**Reighard, J. & Jennings, H. S. (1901). *Anatomy of the Cat*. New York: Henry
Holt & Co.** — 173 original figures (drawn by Louise Burridge Jennings),
including the full skeleton. Published pre-1928 → **public domain worldwide**;
no attribution or share-alike constraints.

- Internet Archive (scans): https://archive.org/details/anatomyofcatrje00reig
- Project Gutenberg (eBook #58394): https://www.gutenberg.org/ebooks/58394
- Biodiversity Heritage Library: https://www.biodiversitylibrary.org/bibliography/1021

This is the authoritative skeletal/muscular reference for the project; it
supersedes the ad-hoc `Felis_silvestris_*.jpg` used earlier (whose license was
not established — it is not tracked in the repo).

## Secondary source (attribution required)

**"Skeleton diagram of a cat", Wikimedia Commons** — labeled lateral skeleton
with the vertebral regions. Licensed **CC BY-SA 3.0** (attribution +
share-alike required *if the image itself is reused*; the plain anatomical
facts below are not copyrightable).
- https://commons.wikimedia.org/wiki/File:Skeleton_diagram_of_a_cat.svg

## Key anatomical facts used by the model

### Vertebral formula (domestic cat)
| Region | Count | Robot mapping |
|--------|-------|---------------|
| Cervical (neck) | 7 | not modelled (head/neck out of scope) |
| **Thoracic** (rib-bearing) | **13** | the **rigid ribcage** region — front of the spine (stiff) |
| **Lumbar** (flexible) | **7** | the **flexible bending** region — rear of the spine; where the arch happens |
| Sacral | 3 | pelvic girdle mount |
| Caudal (tail) | ~19–21 | the long tapering tail (ADR-0007) |

→ Our 3-segment tendon spine ([ADR-0006](../../docs/DESIGN_DECISIONS.md)) is a
coarse abstraction of the ~20 presacral vertebrae, with the articulation
concentrated toward the **lumbar (rear)** third — exactly the CAD's thorax-stiff
/ lumbar-mobile split.

### Digitigrade limbs (cats walk on their toes)
- **Forelimb:** scapula (large, mobile) → humerus → radius/ulna → carpus (wrist)
  → metacarpus → phalanges. Held relatively **columnar**; elbow points back.
- **Hindlimb:** femur → tibia/fibula → tarsus (the **hock/calcaneus**, held
  **high**) → metatarsus → phalanges. Strongly **folded** (the propulsion Z);
  stifle (knee) points forward, hock points back — i.e. fore and hind fold in
  **opposite** directions.
- The long metatarsus/metacarpus is near-vertical in stance; only the toes
  (phalanges) touch the ground → our **4-link leg** = femur/tibia/metatarsus/paw
  with the paw a passive ground-contact link.
- **No functional clavicle** → large shoulder range of motion; contributes to the
  righting reflex ([ADR-0007](../../docs/DESIGN_DECISIONS.md)).

## How this maps to the design
| Anatomy | Design element |
|---------|----------------|
| 13 thoracic + ribcage / 7 lumbar | rigid front vs flexible rear spine (CAD ribcage; ADR-0006) |
| Digitigrade fore vs hind limbs | `DEFAULT_FORELEG` (columnar) vs `DEFAULT_LEG` (folded hind); 4-link digitigrade leg |
| Opposite fore/hind fold | front legs mirror rear in the CAD |
| Long caudal tail | single-tendon tail (ADR-0007) |

*Placeholder dimensions in the model are informed by, not measured from, these
sources; treat proportions as biomimetic approximations pending mechanical
design.*
