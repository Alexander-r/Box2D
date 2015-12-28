#pragma once

#include "stdafx.h"
#include "Body.cpp"
#include "Vector.cpp"

using namespace System::Runtime::InteropServices;

namespace Box2D
{
	namespace Net
	{
		//TODO: is there a way to automatically include the b2JointType as an enum here?
		public enum class JointType
		{
			e_unknownJoint = ::e_unknownJoint,
			e_revoluteJoint = ::e_revoluteJoint,
			e_prismaticJoint = ::e_prismaticJoint,

			e_distanceJoint = ::e_distanceJoint,
			e_pulleyJoint = ::e_pulleyJoint,
			e_mouseJoint = ::e_mouseJoint,
			e_gearJoint = ::e_gearJoint
		};

		public ref class JointDef
		{
		internal:
			b2JointDef *def;
			JointDef() : def(new b2JointDef()) { }
			JointDef(b2JointDef *justBuilt) : def(justBuilt) { }
			virtual ~JointDef()
			{
				delete def;
			}

		public:
			property JointType Type
			{
				JointType get()
				{
					return (JointType)def->type;
				}

				void set(JointType value)
				{
					def->type = (b2JointType)value;
				}
			}

			property Object^ UserData
			{
				Object^ get()
				{
					return System::Runtime::InteropServices::GCHandle::FromIntPtr((System::IntPtr)def->userData).Target;
				}

				void set(Object^ value)
				{
					GCHandle^ gch = GCHandle::Alloc(value);					
					def->userData = (void *)gch->ToIntPtr(*gch);
				}
			}

			property Body^ Body1
			{
				Body^ get()
				{
					return gcnew Body(def->body1);
				}

				void set(Body^ value)
				{
					def->body1 = value->body;
				}
			}

			property Body^ Body2
			{
				Body^ get()
				{
					return gcnew Body(def->body2);
				}

				void set(Body^ value)
				{
					def->body2 = value->body;
				}
			}

			property bool CollideConnected
			{
				bool get()
				{
					return def->collideConnected;
				}

				void set(bool value)
				{
					def->collideConnected = value;
				}
			}
		};

		public ref class MouseJointDef : public JointDef
		{
		internal:
			b2MouseJointDef *mouseJoint;
		public:
			MouseJointDef() : JointDef(mouseJoint = new b2MouseJointDef()) { }

			property Vector^ Target
			{
				Vector^ get()
				{
					return gcnew Vector(mouseJoint->target);
				}

				void set(Vector^ value)
				{
					mouseJoint->target = value->getVec2();
				}
			}

			property float MaxForce
			{
				float get()
				{
					return mouseJoint->maxForce;
				}

				void set(float value)
				{
					mouseJoint->maxForce = value;
				}
			}

			property float FrequencyHz
			{
				float get()
				{
					return mouseJoint->frequencyHz;
				}

				void set(float value)
				{
					mouseJoint->frequencyHz = value;
				}
			}

			property float DampingRatio
			{
				float get()
				{
					return mouseJoint->dampingRatio;
				}

				void set(float value)
				{
					mouseJoint->dampingRatio = value;
				}
			}

			property float TimeStep
			{
				float get()
				{
					return mouseJoint->timeStep;
				}

				void set(float value)
				{
					mouseJoint->timeStep = value;
				}
			}
		};
	}
}
