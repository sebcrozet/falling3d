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
import Control.Monad
import Physics.Falling.Collision.Detection.NarrowPhase
import Physics.Falling.RigidBody.Positionable
import Physics.Falling.RigidBody.CollisionVolume
import Physics.Falling.RigidBody.RigidBody
import Physics.Falling.RigidBody.OrderedRigidBody
import Physics.Falling.Collision.Detection.BallBallCollisionDetector
import Physics.Falling.Collision.Detection.PlaneImplicitShapeCollisionDetector
import Physics.Falling.Collision.Detection.ImplicitShapeImplicitShapeCollisionDetector
import Physics.Falling.Collision.Detection.IncrementalContactManifold
import Physics.Falling.Collision.Collision
import Physics.Falling3d.UnitSphere3d()
import Physics.Falling3d.OrthonormalBasis3d()
import Physics.Falling3d.Transform3d
import Physics.Falling3d.RigidBody3d
import Physics.Falling3d.Shape3d
import Physics.Falling3d.Collision3d

data Shape3dNarrowPhase = Shape3dNarrowPhase ContactManifold3d
                          deriving(Show)


instance Ord idt => NarrowPhase Shape3dNarrowPhase (OrderedRigidBody3d idt) ContactManifold3d where
  update        iorb1 iorb2 (Shape3dNarrowPhase cm) = Shape3dNarrowPhase
                                                      $ collideIndexedOrderedRigidBodies iorb1 iorb2 cm
  collisions    (Shape3dNarrowPhase cm) = cm
  numCollisions (Shape3dNarrowPhase cm) = length cm

-- rigid body <-> rigid body collision dispatch
collideIndexedOrderedRigidBodies :: Ord idt =>
                                    (Int, OrderedRigidBody3d idt) ->
                                    (Int, OrderedRigidBody3d idt) ->
                                    ContactManifold3d             ->
                                    ContactManifold3d
collideIndexedOrderedRigidBodies (id1, orb1) (id2, orb2) =
                                 collideIndexedRigidBodies id1 id2 (rigidBody orb1) (rigidBody orb2)

collideIndexedRigidBodies :: Int -> Int -> RigidBody3d -> RigidBody3d -> ContactManifold3d -> ContactManifold3d
collideIndexedRigidBodies _ id2 (StaticBody  sb) (DynamicBody db) cm = do
                          collision <- collideStaticDynamicShapes ((worldSpaceCollisionVolume sb)
                                                                   , (localToWorld  sb)
                                                                   , (worldToLocal  sb))
                                                                  ((worldSpaceCollisionVolume db)
                                                                   , (localToWorld  db)
                                                                   , (worldToLocal  db))
                                                                  (contactManifoldGeometries cm)
                          return $ collisionDescr2UnibodyCollision id2 collision
collideIndexedRigidBodies id1 id2 db@(DynamicBody _) sb@(StaticBody  _) cm = collideIndexedRigidBodies id2 id1 sb db cm
collideIndexedRigidBodies id1 id2 (DynamicBody db1) (DynamicBody db2) cm =
                          do
                          collision <- collideDynamicDynamicShapes ((worldSpaceCollisionVolume db1)
                                                                    , (localToWorld  db1)
                                                                    , (worldToLocal  db1))
                                                                   ((worldSpaceCollisionVolume db2)
                                                                    , (localToWorld  db2)
                                                                    , (worldToLocal  db2))
                                                                  (contactManifoldGeometries cm)
                          return $ collisionDescr2BibodyCollision id1 id2 collision
collideIndexedRigidBodies _ _ (StaticBody  _) (StaticBody _) _ = error "Cannot collide two static bodies."

-- shape <-> shape collision dispatch
collideStaticDynamicShapes :: (TransformedStaticShape3d,  Transform3d, Transform3d) ->
                              (TransformedDynamicShape3d, Transform3d, Transform3d) ->
                              [ CollisionDescr3d ]                       ->
                              [ CollisionDescr3d ]
collideStaticDynamicShapes ((TransformedStaticBall3d b1), _, it1) ((TransformedBall3d b2), _, it2) _ =
                           _maybe2List
                           $ liftM (mkCollisionDescr it1 it2)
                           $ collideBallBall b1 b2

collideStaticDynamicShapes ((TransformedStaticBall3d b), _, it1) ((TransformedBox3d r), _, it2) _ =
                            _maybe2List
                            $ liftM (mkCollisionDescr it1 it2)
                            $ collideImplicitShapeImplicitShape b r _subdivisionNumber

collideStaticDynamicShapes ((TransformedStaticBox3d r), _, it1) ((TransformedBall3d b), _, it2) _ =
                            _maybe2List
                            $ liftM (mkCollisionDescr it1 it2)
                            $ collideImplicitShapeImplicitShape r b _subdivisionNumber

collideStaticDynamicShapes ((TransformedStaticBox3d r1), t1, it1) ((TransformedBox3d r2), t2, it2) cm =
                            _updateAdd t1 t2 cm
                            $ liftM (mkCollisionDescr it1 it2)
                            $ collideImplicitShapeImplicitShape r1 r2 _subdivisionNumber

collideStaticDynamicShapes ((TransformedPlane3d p), _, it1) ((TransformedBall3d b), _, it2) _ =
                           _maybe2List
                           $ liftM (mkCollisionDescr it1 it2)
                           $ collidePlaneImplicitShape p b

collideStaticDynamicShapes ((TransformedPlane3d p), t1, it1) ((TransformedBox3d r), t2, it2) cm =
                            _updateAdd t1 t2 cm
                           $ liftM (mkCollisionDescr it1 it2)
                           $ collidePlaneImplicitShape p r


collideDynamicDynamicShapes :: (TransformedDynamicShape3d, Transform3d, Transform3d) ->
                               (TransformedDynamicShape3d, Transform3d, Transform3d) ->
                               [ CollisionDescr3d ]                       ->
                               [ CollisionDescr3d ]
collideDynamicDynamicShapes ((TransformedBall3d b1), _, it1)        ((TransformedBall3d b2), _, it2) _ =
                            _maybe2List
                            $ liftM (mkCollisionDescr it1 it2)
                            $ collideBallBall b1 b2

collideDynamicDynamicShapes ((TransformedBox3d r), _, it1) ((TransformedBall3d b), _, it2) _ = 
                            _maybe2List
                            $ liftM (mkCollisionDescr it1 it2)
                            $ collideImplicitShapeImplicitShape r b _subdivisionNumber

collideDynamicDynamicShapes ((TransformedBox3d r1), t1, it1) ((TransformedBox3d r2), t2, it2) cm = 
                            _updateAdd t1 t2 cm
                            $ liftM (mkCollisionDescr it1 it2)
                            $ collideImplicitShapeImplicitShape r1 r2 _subdivisionNumber

collideDynamicDynamicShapes s1                   s2                   cm =
                            collideDynamicDynamicShapes s2 s1 rcm >>= return . revertCollisionDescr
                            where
                            rcm = map revertCollisionDescr cm

shape3dCollisionDispatcher :: OrderedRigidBody3d idt -> OrderedRigidBody3d idt -> Shape3dNarrowPhase
shape3dCollisionDispatcher _ _ = Shape3dNarrowPhase []

_subdivisionNumber :: Int
_subdivisionNumber = 42

_maybe2List :: Maybe a -> [a]
_maybe2List Nothing  = []
_maybe2List (Just a) = [a]

_updateAdd :: Transform3d -> Transform3d -> [CollisionDescr3d] -> Maybe CollisionDescr3d -> [CollisionDescr3d]
_updateAdd t1 t2 cm c = case c of
           Nothing   -> updateContacts t1 t2 cm
           Just coll -> addContact coll $ updateContacts t1 t2 cm
