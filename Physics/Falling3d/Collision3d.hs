module Physics.Falling3d.Collision3d
(
CollisionDescr3d
, Collision3d
, ContactManifold3d
)
where

import Data.Vect.Double.Base
import Physics.Falling.Collision.Collision

type CollisionDescr3d  = CollisionDescr  Vec3 Normal3
type Collision3d       = Collision       Vec3 Normal3
type ContactManifold3d = ContactManifold Vec3 Normal3
