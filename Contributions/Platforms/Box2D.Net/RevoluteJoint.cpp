#pragma once

#include "stdafx.h"
#include "Joint.cpp"
#include "JointDef.cpp"

namespace Box2D
{
	namespace Net
	{
		public ref class RevoluteJointDef : public JointDef
		{
		public:
			RevoluteJointDef() : JointDef(new b2RevoluteJointDef()) { }

			property Vector^ LocalAnchor1
			{
				Vector^ get()
				{
					return gcnew Vector(reinterpret_cast<b2RevoluteJointDef *>(def)->localAnchor1);
				}

				void set(Vector^ value)
				{
					reinterpret_cast<b2RevoluteJointDef *>(def)->localAnchor1 = value->getVec2();
				}
			}

			property Vector^ LocalAnchor2
			{
				Vector^ get()
				{
					return gcnew Vector(reinterpret_cast<b2RevoluteJointDef *>(def)->localAnchor2);
				}

				void set(Vector^ value)
				{
					reinterpret_cast<b2RevoluteJointDef *>(def)->localAnchor2 = value->getVec2();
				}
			}

			property float LowerAngle
			{
				float get()
				{
					return reinterpret_cast<b2RevoluteJointDef *>(def)->lowerAngle;
				}

				void set(float value)
				{
					reinterpret_cast<b2RevoluteJointDef *>(def)->lowerAngle = value;
				}
			}

			property float UpperAngle
			{
				float get()
				{
					return reinterpret_cast<b2RevoluteJointDef *>(def)->upperAngle;
				}

				void set(float value)
				{
					reinterpret_cast<b2RevoluteJointDef *>(def)->upperAngle = value;
				}
			}

			property float MotorTorque
			{
				float get()
				{
					return reinterpret_cast<b2RevoluteJointDef *>(def)->maxMotorTorque;
				}

				void set(float value)
				{
					reinterpret_cast<b2RevoluteJointDef *>(def)->maxMotorTorque = value;
				}
			}

			property float MotorSpeed
			{
				float get()
				{
					return reinterpret_cast<b2RevoluteJointDef *>(def)->motorSpeed;
				}

				void set(float value)
				{
					reinterpret_cast<b2RevoluteJointDef *>(def)->motorSpeed = value;
				}
			}

			property bool EnableLimit
			{
				bool get()
				{
					return reinterpret_cast<b2RevoluteJointDef *>(def)->enableLimit;
				}

				void set(bool value)
				{
					reinterpret_cast<b2RevoluteJointDef *>(def)->enableLimit = value;
				}
			}

			property bool EnableMotor
			{
				bool get()
				{
					return reinterpret_cast<b2RevoluteJointDef *>(def)->enableMotor;
				}

				void set(bool value)
				{
					reinterpret_cast<b2RevoluteJointDef *>(def)->enableMotor = value;
				}
			}
			
			void Initialize(Body^ body1, Body^ body2, Vector^ Anchor)
			{
				reinterpret_cast<b2RevoluteJointDef*>(def)->Initialize(body1->body, body2->body, Anchor->getVec2());
			}
		};

		public ref class RevoluteJoint : public Joint
		{
		internal:
			RevoluteJoint(b2RevoluteJoint *jointRef) : Joint(jointRef) { }

		public:
			property Vector^ Anchor1
			{
				Vector^ get()
				{
					return gcnew Vector(reinterpret_cast<b2RevoluteJoint*>(joint)->GetAnchor1());
				}
			}

			property Vector^ Anchor2
			{
				Vector^ get()
				{
					return gcnew Vector(reinterpret_cast<b2RevoluteJoint*>(joint)->GetAnchor2());
				}
			}

			Vector^ GetReactionForce()
			{
				return gcnew Vector(reinterpret_cast<b2RevoluteJoint*>(joint)->GetReactionForce());
			}

			float GetReactionTorque()
			{
				return (reinterpret_cast<b2RevoluteJoint*>(joint)->GetReactionTorque());
			}

			property float JointAngle
			{
				float get()
				{
					return (reinterpret_cast<b2RevoluteJoint*>(joint)->GetJointAngle());
				}
			}

			property float JointSpeed
			{
				float get()
				{
					return (reinterpret_cast<b2RevoluteJoint*>(joint)->GetJointSpeed());
				}
			}

			float GetMotorTorque()
			{
				return reinterpret_cast<b2RevoluteJoint*>(joint)->GetMotorTorque();
			}

			void SetMotorSpeed(float speed)
			{
				reinterpret_cast<b2RevoluteJoint*>(joint)->SetMotorSpeed(speed);
			}

			void SetMotorTorque(float torque)
			{
				reinterpret_cast<b2RevoluteJoint*>(joint)->SetMaxMotorTorque(torque);
			}
		};
	}
}
