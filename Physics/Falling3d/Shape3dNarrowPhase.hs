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

import Physics.Falling.Collision.Detection.NarrowPhase
import Physics.Falling.RigidBody.Positionable
import Physics.Falling.RigidBody.CollisionVolume
import Physics.Falling.RigidBody.RigidBody
import Physics.Falling.RigidBody.OrderedRigidBody
import Physics.Falling.Collision.Detection.BallBallCollisionDetector
import Physics.Falling.Collision.Detection.PlaneImplicitShapeCollisionDetector
import Physics.Falling.Collision.Collision
import Physics.Falling3d.Transform3d
import Physics.Falling3d.RigidBody3d
import Physics.Falling3d.Shape3d
import Physics.Falling3d.Collision3d

data Shape3dNarrowPhase = Shape3dNarrowPhase ContactManifold3d


instance Ord idt => NarrowPhase Shape3dNarrowPhase (OrderedRigidBody3d idt) ContactManifold3d where
  update        iorb1 iorb3 _           = Shape3dNarrowPhase $ collideIndexedOrderedRigidBodies iorb1 iorb3
  collisions    (Shape3dNarrowPhase cm) = cm
  numCollisions (Shape3dNarrowPhase cm) = length cm

collideIndexedOrderedRigidBodies :: Ord idt =>
                                    (Int, OrderedRigidBody3d idt) -> (Int, OrderedRigidBody3d idt) -> ContactManifold3d
collideIndexedOrderedRigidBodies (id1, orb1) (id3, orb3) =
                                 case collideIndexedRigidBodies id1 id3 (rigidBody orb1) (rigidBody orb3) of
                                   Nothing   -> []
                                   Just    c -> [c]

collideIndexedRigidBodies :: Int -> Int -> RigidBody3d -> RigidBody3d -> Maybe Collision3d
collideIndexedRigidBodies _ id3 (StaticBody  sb) (DynamicBody db) = do
                          collision <- collideStaticDynamicShapes (getCollisionVolume sb)
                                                                  (getCollisionVolume db)
                                                                  (getLocalToWorld    sb)
                                                                  (getLocalToWorld    db)
                                                                  (getWorldToLocal    db)
                          return $ collisionDescr2UnibodyCollision id3 collision
collideIndexedRigidBodies id1 id3 db@(DynamicBody _) sb@(StaticBody  _) = collideIndexedRigidBodies id3 id1 sb db
collideIndexedRigidBodies id1 id3 (DynamicBody db1) (DynamicBody db3) = do
                          collision <- collideDynamicDynamicShapes (getCollisionVolume db1)
                                                                   (getCollisionVolume db3)
                                                                   (getLocalToWorld    db1)
                                                                   (getLocalToWorld    db3)
                          return $ collisionDescr2BibodyCollision id1 id3 collision
collideIndexedRigidBodies _ _ (StaticBody  _) (StaticBody _) = error "Cannot collide two static bodies."

collideStaticDynamicShapes :: StaticShape3d -> DynamicShape3d ->
                              Transform3d -> Transform3d -> Transform3d ->
                              Maybe CollisionDescr3d
collideStaticDynamicShapes (StaticBall3d b1) (Ball3d b3) st dt _   = collideBallBall b1 b3 st dt
collideStaticDynamicShapes (Plane3d      p)  (Ball3d b)  st dt idt = collidePlaneImplicitShape p b st dt idt

-- collideDynamicStaticShapes :: DynamicShape3d -> StaticShape3d ->
--                               Transform3d -> Transform3d -> Transform3d ->
--                               Maybe CollisionDescr3d
-- collideDynamicStaticShapes ds ss dt idt st = collideStaticDynamicShapes ss ds st dt idt

collideDynamicDynamicShapes :: DynamicShape3d -> DynamicShape3d -> Transform3d -> Transform3d -> Maybe CollisionDescr3d
collideDynamicDynamicShapes (Ball3d b1) (Ball3d b3) t1 t3 = collideBallBall b1 b3 t1 t3

shape3dCollisionDispatcher :: OrderedRigidBody3d idt -> OrderedRigidBody3d idt -> Shape3dNarrowPhase
shape3dCollisionDispatcher _ _ = Shape3dNarrowPhase []
