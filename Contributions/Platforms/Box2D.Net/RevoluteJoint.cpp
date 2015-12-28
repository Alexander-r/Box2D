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

			property double LowerAngle
			{
				double get()
				{
					return reinterpret_cast<b2RevoluteJointDef *>(def)->lowerAngle;
				}

				void set(double value)
				{
					reinterpret_cast<b2RevoluteJointDef *>(def)->lowerAngle = value;
				}
			}

			property double UpperAngle
			{
				double get()
				{
					return reinterpret_cast<b2RevoluteJointDef *>(def)->upperAngle;
				}

				void set(double value)
				{
					reinterpret_cast<b2RevoluteJointDef *>(def)->upperAngle = value;
				}
			}

			property double MotorTorque
			{
				double get()
				{
					return reinterpret_cast<b2RevoluteJointDef *>(def)->maxMotorTorque;
				}

				void set(double value)
				{
					reinterpret_cast<b2RevoluteJointDef *>(def)->maxMotorTorque = value;
				}
			}

			property double MotorSpeed
			{
				double get()
				{
					return reinterpret_cast<b2RevoluteJointDef *>(def)->motorSpeed;
				}

				void set(double value)
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

			double GetReactionTorque()
			{
				return (reinterpret_cast<b2RevoluteJoint*>(joint)->GetReactionTorque());
			}

			property double JointAngle
			{
				double get()
				{
					return (reinterpret_cast<b2RevoluteJoint*>(joint)->GetJointAngle());
				}
			}

			property double JointSpeed
			{
				double get()
				{
					return (reinterpret_cast<b2RevoluteJoint*>(joint)->GetJointSpeed());
				}
			}

			double GetMotorTorque()
			{
				return reinterpret_cast<b2RevoluteJoint*>(joint)->GetMotorTorque();
			}

			void SetMotorSpeed(double speed)
			{
				reinterpret_cast<b2RevoluteJoint*>(joint)->SetMotorSpeed(speed);
			}

			void SetMotorTorque(double torque)
			{
				reinterpret_cast<b2RevoluteJoint*>(joint)->SetMaxMotorTorque(torque);
			}
		};
	}
}
