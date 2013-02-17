module Physics.Falling3d.RigidBody3d
(
RigidBody3d
, OrderedRigidBody3d
)
where

import Data.Vect.Double.Base
import Physics.Falling.RigidBody.RigidBody
import Physics.Falling.RigidBody.OrderedRigidBody
import Physics.Falling3d.InertiaTensor3d
import Physics.Falling3d.Shape3d
import Physics.Falling3d.Transform3d

type RigidBody3d = RigidBody Transform3d
                             Vec3
                             Vec3
                             InertiaTensor3d
                             InverseInertiaTensor3d
                             DynamicShape3d
                             StaticShape3d
                             TransformedDynamicShape3d
                             TransformedStaticShape3d

type OrderedRigidBody3d identifierType = OrderedRigidBody identifierType
                                                          Transform3d
                                                          Vec3
                                                          Vec3
                                                          InertiaTensor3d
                                                          InverseInertiaTensor3d
                                                          DynamicShape3d
                                                          StaticShape3d
                                                          TransformedDynamicShape3d
                                                          TransformedStaticShape3d
