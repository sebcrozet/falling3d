module Physics.Falling3d.World3d
(
World3d
, DefaultWorld3d
, mkWorld3d
)
where

import Data.Vect.Double.Base
import qualified Physics.Falling.World.WorldWithRigidBody as W
import Physics.Falling.Collision.Detection.BruteForceBroadPhase
import Physics.Falling.RigidBody.RigidBodySolver
import Physics.Falling3d.RigidBody3d
import Physics.Falling3d.InertiaTensor3d
import Physics.Falling3d.Transform3d
import Physics.Falling3d.Shape3d
import Physics.Falling3d.Shape3dNarrowPhase
import Physics.Falling3d.Collision3d

type World3d identifierType broadPhaseType narrowPhaseType contactManifoldType = W.World Transform3d
                                                                                         Vec3
                                                                                         Vec3
                                                                                         InertiaTensor3d
                                                                                         InverseInertiaTensor3d
                                                                                         DynamicShape3d
                                                                                         StaticShape3d
                                                                                         TransformedDynamicShape3d
                                                                                         TransformedStaticShape3d
                                                                                         broadPhaseType
                                                                                         narrowPhaseType
                                                                                         contactManifoldType
                                                                                         identifierType

type DefaultWorld3d identifierType = World3d identifierType
                                             (BruteForceBroadPhase (OrderedRigidBody3d identifierType))
                                             Shape3dNarrowPhase
                                             ContactManifold3d

-- | Creates a physics world able to simulate 3D physics. It uses the default broad phase
-- (currently a brute-force broad phase).
mkWorld3d :: Ord identifierType => DefaultWorld3d identifierType
mkWorld3d = W.mkWorld mkBroadPhase
                      shape3dCollisionDispatcher
                      solveConstraintsIsland
