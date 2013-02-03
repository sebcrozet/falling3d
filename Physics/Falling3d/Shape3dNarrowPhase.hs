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
import Physics.Falling.Collision.Detection.IncrementalContactManifold
import Physics.Falling.Collision.Collision
import Physics.Falling3d.UnitSphere3d()
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
                          collision <- collideStaticDynamicShapes ((getCollisionVolume sb)
                                                                   , (getLocalToWorld  sb)
                                                                   , (getWorldToLocal  sb))
                                                                  ((getCollisionVolume db)
                                                                   , (getLocalToWorld  db)
                                                                   , (getWorldToLocal  db))
                                                                  (contactManifoldGeometries cm)
                          return $ collisionDescr2UnibodyCollision id2 collision
collideIndexedRigidBodies id1 id2 db@(DynamicBody _) sb@(StaticBody  _) cm = collideIndexedRigidBodies id2 id1 sb db cm
collideIndexedRigidBodies id1 id2 (DynamicBody db1) (DynamicBody db2) cm = do
                          collision <- collideDynamicDynamicShapes ((getCollisionVolume db1)
                                                                    , (getLocalToWorld  db1)
                                                                    , (getWorldToLocal  db1))
                                                                   ((getCollisionVolume db2)
                                                                    , (getLocalToWorld  db2)
                                                                    , (getWorldToLocal  db2))
                                                                  (contactManifoldGeometries cm)
                          return $ collisionDescr2BibodyCollision id1 id2 collision
collideIndexedRigidBodies _ _ (StaticBody  _) (StaticBody _) _ = error "Cannot collide two static bodies."

-- shape <-> shape collision dispatch
collideStaticDynamicShapes :: (StaticShape3d,  Transform3d, Transform3d) ->
                              (DynamicShape3d, Transform3d, Transform3d) ->
                              [ CollisionDescr3d ]                       ->
                              [ CollisionDescr3d ]
collideStaticDynamicShapes ((StaticBall3d b1), t1, _) ((Ball3d b2), t2, _) _ =
                           _maybe2List $ collideBallBall b1 b2 t1 t2

collideStaticDynamicShapes ((StaticBall3d b), t1, it1) ((Box3d r), t2, it2) _ =
                            _maybe2List $ collideImplicitShapeImplicitShape (b, t1, it1)
                                                                            (r, t2, it2)
                                                                            _subdivisionNumber

collideStaticDynamicShapes ((StaticBox3d r), t1, it1) ((Ball3d b), t2, it2) _ =
                            _maybe2List $ collideImplicitShapeImplicitShape (r, t1, it1)
                                                                            (b, t2, it2)
                                                                            _subdivisionNumber

collideStaticDynamicShapes ((StaticBox3d r1), t1, it1) ((Box3d r2), t2, it2) cm =
                            _updateAdd (t1, it1) (t2, it2) cm
                            $ collideImplicitShapeImplicitShape (r1, t1, it1)
                                                                (r2, t2, it2)
                                                                _subdivisionNumber

collideStaticDynamicShapes ((Plane3d p), t1, it1) ((Ball3d b), t2, it2) _ =
                           _maybe2List $ collidePlaneImplicitShape (p, t1, it1) (b, t2, it2)

collideStaticDynamicShapes ((Plane3d p), t1, it1) ((Box3d r), t2, it2) cm =
                            _updateAdd (t1, it1) (t2, it2) cm
                           $ collidePlaneImplicitShape (p, t1, it1) (r, t2, it2)


collideDynamicDynamicShapes :: (DynamicShape3d, Transform3d, Transform3d) ->
                               (DynamicShape3d, Transform3d, Transform3d) ->
                               [ CollisionDescr3d ]                       ->
                               [ CollisionDescr3d ]
collideDynamicDynamicShapes ((Ball3d b1), t1, _)        ((Ball3d b2), t2, _) _ =
                            _maybe2List $ collideBallBall b1 b2 t1 t2

collideDynamicDynamicShapes ((Ball3d b), t1, it1)       ((Box3d r), t2, it2) _ = 
                            _maybe2List $ collideImplicitShapeImplicitShape (b, t1, it1)
                                                                            (r, t2, it2)
                                                                            _subdivisionNumber

collideDynamicDynamicShapes ((Box3d r1), t1, it1) ((Box3d r2), t2, it2) cm = 
                            _updateAdd (t1, it1) (t2, it2) cm
                            $  collideImplicitShapeImplicitShape (r1, t1, it1)
                                                                 (r2, t2, it2)
                                                                 _subdivisionNumber

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

_updateAdd :: (Transform3d, Transform3d) ->
              (Transform3d, Transform3d) ->
              [CollisionDescr3d]         ->
              Maybe CollisionDescr3d     ->
              [CollisionDescr3d]
_updateAdd t1s t2s cm c = case c of
           Nothing   -> updateContacts t1s t2s cm
           Just coll -> addContact coll $ updateContacts t1s t2s cm
