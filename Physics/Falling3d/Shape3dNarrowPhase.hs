{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE TypeSynonymInstances  #-}
{-# LANGUAGE FlexibleInstances     #-}
{-# LANGUAGE UndecidableInstances  #-}

module Physics.Falling3d.Shape3dNarrowPhase
(
Shape3dNarrowPhase
, shape3dCollisionDispatcher
)
where

import Data.Vect.Double.Instances()
import Physics.Falling.Collision.Detection.NarrowPhase
import Physics.Falling.RigidBody.Positionable
import Physics.Falling.RigidBody.CollisionVolume
import Physics.Falling.RigidBody.RigidBody
import Physics.Falling.RigidBody.OrderedRigidBody
import Physics.Falling.Collision.Detection.BallBallCollisionDetector
import Physics.Falling.Collision.Detection.PlaneImplicitShapeCollisionDetector
import Physics.Falling.Collision.Detection.ImplicitShapeImplicitShapeCollisionDetector
import Physics.Falling.Collision.Collision
import Physics.Falling.Shape.ShapeWithMargin
import Physics.Falling3d.UnitSphere3d()
import Physics.Falling3d.Transform3d
import Physics.Falling3d.RigidBody3d
import Physics.Falling3d.Shape3d
import Physics.Falling3d.Collision3d

data Shape3dNarrowPhase = Shape3dNarrowPhase ContactManifold3d
                          deriving(Show)


instance Ord idt => NarrowPhase Shape3dNarrowPhase (OrderedRigidBody3d idt) ContactManifold3d where
  update        iorb1 iorb2 _           = Shape3dNarrowPhase $ collideIndexedOrderedRigidBodies iorb1 iorb2
  collisions    (Shape3dNarrowPhase cm) = cm
  numCollisions (Shape3dNarrowPhase cm) = length cm

-- rigid body <-> rigid body collision dispatch
collideIndexedOrderedRigidBodies :: Ord idt =>
                                    (Int, OrderedRigidBody3d idt) -> (Int, OrderedRigidBody3d idt) -> ContactManifold3d
collideIndexedOrderedRigidBodies (id1, orb1) (id2, orb2) =
                                 case collideIndexedRigidBodies id1 id2 (rigidBody orb1) (rigidBody orb2) of
                                   Nothing   -> []
                                   Just    c -> [c]

collideIndexedRigidBodies :: Int -> Int -> RigidBody3d -> RigidBody3d -> Maybe Collision3d
collideIndexedRigidBodies _ id2 (StaticBody  sb) (DynamicBody db) = do
                          collision <- collideStaticDynamicShapes ((getCollisionVolume sb)
                                                                   , (getLocalToWorld  sb)
                                                                   , (getWorldToLocal  sb))
                                                                  ((getCollisionVolume db)
                                                                   , (getLocalToWorld  db)
                                                                   , (getWorldToLocal  db))
                          return $ collisionDescr2UnibodyCollision id2 collision
collideIndexedRigidBodies id1 id2 db@(DynamicBody _) sb@(StaticBody  _) = collideIndexedRigidBodies id2 id1 sb db
collideIndexedRigidBodies id1 id2 (DynamicBody db1) (DynamicBody db2) = do
                          collision <- collideDynamicDynamicShapes ((getCollisionVolume db1)
                                                                    , (getLocalToWorld  db1)
                                                                    , (getWorldToLocal  db1))
                                                                   ((getCollisionVolume db2)
                                                                    , (getLocalToWorld  db2)
                                                                    , (getWorldToLocal  db2))
                          return $ collisionDescr2BibodyCollision id1 id2 collision
collideIndexedRigidBodies _ _ (StaticBody  _) (StaticBody _) = error "Cannot collide two static bodies."

-- shape <-> shape collision dispatch
collideStaticDynamicShapes :: (StaticShape3d,  Transform3d, Transform3d) ->
                              (DynamicShape3d, Transform3d, Transform3d) ->
                              Maybe CollisionDescr3d
collideStaticDynamicShapes ((StaticBall3d b1), t1, _) ((Ball3d b2), t2, _) =
                           collideBallBall b1 b2 t1 t2

collideStaticDynamicShapes ((StaticBall3d b), t1, it1) ((Box3d r), t2, it2) =
                            collideImplicitShapeImplicitShape (b, t1, it1) (r, t2, it2) _subdivisionNumber

collideStaticDynamicShapes ((StaticBox3d r), t1, it1) ((Ball3d b), t2, it2) =
                            collideImplicitShapeImplicitShape (r, t1, it1) (b, t2, it2) _subdivisionNumber

collideStaticDynamicShapes ((StaticBox3d r1), t1, it1) ((Box3d r2), t2, it2) =
                            collideImplicitShapeImplicitShape (r1, t1, it1) (r2, t2, it2) _subdivisionNumber

collideStaticDynamicShapes ((Plane3d p), t1, _)        ((Ball3d b), t2, _) =
                           collidePlaneImplicitShape p (ShapeWithMargin b) t1 t2

collideStaticDynamicShapes ((Plane3d p), t1, _)        ((Box3d r), t2, _) =
                           collidePlaneImplicitShape p (ShapeWithMargin r) t1 t2


collideDynamicDynamicShapes :: (DynamicShape3d, Transform3d, Transform3d) ->
                               (DynamicShape3d, Transform3d, Transform3d) ->
                               Maybe CollisionDescr3d
collideDynamicDynamicShapes ((Ball3d b1), t1, _)        ((Ball3d b2), t2, _) =
                            collideBallBall b1 b2 t1 t2

collideDynamicDynamicShapes ((Ball3d b), t1, it1)       ((Box3d r), t2, it2) = 
                            collideImplicitShapeImplicitShape (b, t1, it1) (r, t2, it2) _subdivisionNumber

collideDynamicDynamicShapes ((Box3d r1), t1, it1) ((Box3d r2), t2, it2) = 
                            collideImplicitShapeImplicitShape (r1, t1, it1) (r2, t2, it2) _subdivisionNumber

collideDynamicDynamicShapes s1                   s2                   =
                            collideDynamicDynamicShapes s2 s1 >>= return . revertCollisionDescr

shape3dCollisionDispatcher :: OrderedRigidBody3d idt -> OrderedRigidBody3d idt -> Shape3dNarrowPhase
shape3dCollisionDispatcher _ _ = Shape3dNarrowPhase []

_subdivisionNumber :: Int
_subdivisionNumber = 42
