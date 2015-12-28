#pragma once

#include "stdafx.h"
#include "Vector.cpp"

namespace Box2D
{
	namespace Net
	{
		public ref class MassData
		{
		internal:
			bool DeleteWhenDone;
			b2MassData *data;
			MassData(b2MassData *dataRef) : data(dataRef), DeleteWhenDone(false) { }
		public:
			MassData() : data(new b2MassData()), DeleteWhenDone(true) { }
			virtual ~MassData()
			{
				if(DeleteWhenDone)
					delete data;
			}

			property float Mass
			{
				float get()
				{
					return data->mass;
				}

				void set(float value)
				{
					data->mass = value;
				}
			}

			property float I
			{
				float get()
				{
					return data->I;
				}

				void set(float value)
				{
					data->I = value;
				}
			}

			property Vector^ Center
			{
				Vector^ get()
				{
					return gcnew Vector(data->center);
				}

				void set(Vector^ value)
				{
					data->center = value->getVec2();
				}
			}
		};
	}
}
