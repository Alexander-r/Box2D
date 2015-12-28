#include "BipedDef.h"

int16_t BipedDef::count = 0;

const double k_scale = 3.0;

BipedDef::BipedDef()
{
	SetMotorTorque(2.0);
	SetMotorSpeed(0.0);
	SetDensity(20.0);
	SetRestitution(0.0);
	SetLinearDamping(0.0);
	SetAngularDamping(0.005);
	SetGroupIndex(--count);
	EnableMotor();
	EnableLimit();
	
	DefaultVertices();
	DefaultPositions();
	DefaultJoints();

	LFootPoly.friction = RFootPoly.friction = 0.85;
}

void BipedDef::IsFast(bool b)
{
	B2_NOT_USED(b);
	/*
	LFootDef.isFast			= b;
	RFootDef.isFast			= b;
	LCalfDef.isFast			= b;
	RCalfDef.isFast			= b;
	LThighDef.isFast		= b;
	RThighDef.isFast		= b;
	PelvisDef.isFast		= b;
	StomachDef.isFast		= b;
	ChestDef.isFast			= b;
	NeckDef.isFast			= b;
	HeadDef.isFast			= b;
	LUpperArmDef.isFast		= b;
	RUpperArmDef.isFast		= b;
	LForearmDef.isFast		= b;
	RForearmDef.isFast		= b;
	LHandDef.isFast			= b;
	RHandDef.isFast			= b;
	*/
}

void BipedDef::SetGroupIndex(int16_t i)
{
	LFootPoly.filter.groupIndex		= i;
	RFootPoly.filter.groupIndex		= i;
	LCalfPoly.filter.groupIndex		= i;
	RCalfPoly.filter.groupIndex		= i;
	LThighPoly.filter.groupIndex		= i;
	RThighPoly.filter.groupIndex		= i;
	PelvisPoly.filter.groupIndex		= i;
	StomachPoly.filter.groupIndex		= i;
	ChestPoly.filter.groupIndex		= i;
	NeckPoly.filter.groupIndex			= i;
	HeadCirc.filter.groupIndex			= i;
	LUpperArmPoly.filter.groupIndex	= i;
	RUpperArmPoly.filter.groupIndex	= i;
	LForearmPoly.filter.groupIndex		= i;
	RForearmPoly.filter.groupIndex		= i;
	LHandPoly.filter.groupIndex		= i;
	RHandPoly.filter.groupIndex		= i;
}

void BipedDef::SetLinearDamping(double f)
{
	LFootDef.linearDamping		= f;
	RFootDef.linearDamping		= f;
	LCalfDef.linearDamping		= f;
	RCalfDef.linearDamping		= f;
	LThighDef.linearDamping		= f;
	RThighDef.linearDamping		= f;
	PelvisDef.linearDamping		= f;
	StomachDef.linearDamping	= f;
	ChestDef.linearDamping		= f;
	NeckDef.linearDamping		= f;
	HeadDef.linearDamping		= f;
	LUpperArmDef.linearDamping	= f;
	RUpperArmDef.linearDamping	= f;
	LForearmDef.linearDamping	= f;
	RForearmDef.linearDamping	= f;
	LHandDef.linearDamping		= f;
	RHandDef.linearDamping		= f;
}

void BipedDef::SetAngularDamping(double f)
{
	LFootDef.angularDamping		= f;
	RFootDef.angularDamping		= f;
	LCalfDef.angularDamping		= f;
	RCalfDef.angularDamping		= f;
	LThighDef.angularDamping	= f;
	RThighDef.angularDamping	= f;
	PelvisDef.angularDamping	= f;
	StomachDef.angularDamping	= f;
	ChestDef.angularDamping		= f;
	NeckDef.angularDamping		= f;
	HeadDef.angularDamping		= f;
	LUpperArmDef.angularDamping	= f;
	RUpperArmDef.angularDamping	= f;
	LForearmDef.angularDamping	= f;
	RForearmDef.angularDamping	= f;
	LHandDef.angularDamping		= f;
	RHandDef.angularDamping		= f;
}

void BipedDef::SetMotorTorque(double f)
{
	LAnkleDef.maxMotorTorque		= f;
	RAnkleDef.maxMotorTorque		= f;
	LKneeDef.maxMotorTorque		= f;
	RKneeDef.maxMotorTorque		= f;
	LHipDef.maxMotorTorque			= f;
	RHipDef.maxMotorTorque			= f;
	LowerAbsDef.maxMotorTorque		= f;
	UpperAbsDef.maxMotorTorque		= f;
	LowerNeckDef.maxMotorTorque	= f;
	UpperNeckDef.maxMotorTorque	= f;
	LShoulderDef.maxMotorTorque	= f;
	RShoulderDef.maxMotorTorque	= f;
	LElbowDef.maxMotorTorque		= f;
	RElbowDef.maxMotorTorque		= f;
	LWristDef.maxMotorTorque		= f;
	RWristDef.maxMotorTorque		= f;
}

void BipedDef::SetMotorSpeed(double f)
{
	LAnkleDef.motorSpeed		= f;
	RAnkleDef.motorSpeed		= f;
	LKneeDef.motorSpeed			= f;
	RKneeDef.motorSpeed			= f;
	LHipDef.motorSpeed			= f;
	RHipDef.motorSpeed			= f;
	LowerAbsDef.motorSpeed		= f;
	UpperAbsDef.motorSpeed		= f;
	LowerNeckDef.motorSpeed		= f;
	UpperNeckDef.motorSpeed		= f;
	LShoulderDef.motorSpeed		= f;
	RShoulderDef.motorSpeed		= f;
	LElbowDef.motorSpeed		= f;
	RElbowDef.motorSpeed		= f;
	LWristDef.motorSpeed		= f;
	RWristDef.motorSpeed		= f;
}

void BipedDef::SetDensity(double f)
{
	LFootPoly.density			= f;
	RFootPoly.density			= f;
	LCalfPoly.density			= f;
	RCalfPoly.density			= f;
	LThighPoly.density			= f;
	RThighPoly.density			= f;
	PelvisPoly.density			= f;
	StomachPoly.density			= f;
	ChestPoly.density			= f;
	NeckPoly.density			= f;
	HeadCirc.density			= f;
	LUpperArmPoly.density		= f;
	RUpperArmPoly.density		= f;
	LForearmPoly.density		= f;
	RForearmPoly.density		= f;
	LHandPoly.density			= f;
	RHandPoly.density			= f;
}

void BipedDef::SetRestitution(double f)
{
	LFootPoly.restitution		= f;
	RFootPoly.restitution		= f;
	LCalfPoly.restitution		= f;
	RCalfPoly.restitution		= f;
	LThighPoly.restitution		= f;
	RThighPoly.restitution		= f;
	PelvisPoly.restitution		= f;
	StomachPoly.restitution		= f;
	ChestPoly.restitution		= f;
	NeckPoly.restitution		= f;
	HeadCirc.restitution		= f;
	LUpperArmPoly.restitution	= f;
	RUpperArmPoly.restitution	= f;
	LForearmPoly.restitution	= f;
	RForearmPoly.restitution	= f;
	LHandPoly.restitution		= f;
	RHandPoly.restitution		= f;
}

void BipedDef::EnableLimit()
{
	SetLimit(true);
}

void BipedDef::DisableLimit()
{
	SetLimit(false);
}

void BipedDef::SetLimit(bool b)
{
	LAnkleDef.enableLimit		= b;
	RAnkleDef.enableLimit		= b;
	LKneeDef.enableLimit		= b;
	RKneeDef.enableLimit		= b;
	LHipDef.enableLimit			= b;
	RHipDef.enableLimit			= b;
	LowerAbsDef.enableLimit		= b;
	UpperAbsDef.enableLimit		= b;
	LowerNeckDef.enableLimit	= b;
	UpperNeckDef.enableLimit	= b;
	LShoulderDef.enableLimit	= b;
	RShoulderDef.enableLimit	= b;
	LElbowDef.enableLimit		= b;
	RElbowDef.enableLimit		= b;
	LWristDef.enableLimit		= b;
	RWristDef.enableLimit		= b;
}

void BipedDef::EnableMotor()
{
	SetMotor(true);
}

void BipedDef::DisableMotor()
{
	SetMotor(false);
}

void BipedDef::SetMotor(bool b)
{
	LAnkleDef.enableMotor		= b;
	RAnkleDef.enableMotor		= b;
	LKneeDef.enableMotor		= b;
	RKneeDef.enableMotor		= b;
	LHipDef.enableMotor			= b;
	RHipDef.enableMotor			= b;
	LowerAbsDef.enableMotor		= b;
	UpperAbsDef.enableMotor		= b;
	LowerNeckDef.enableMotor	= b;
	UpperNeckDef.enableMotor	= b;
	LShoulderDef.enableMotor	= b;
	RShoulderDef.enableMotor	= b;
	LElbowDef.enableMotor		= b;
	RElbowDef.enableMotor		= b;
	LWristDef.enableMotor		= b;
	RWristDef.enableMotor		= b;
}

BipedDef::~BipedDef(void)
{
}

void BipedDef::DefaultVertices()
{
	{	// feet
		LFootPoly.vertexCount = RFootPoly.vertexCount = 5;
		LFootPoly.vertices[0] = RFootPoly.vertices[0] = k_scale * b2Vec2(.033,.143);
		LFootPoly.vertices[1] = RFootPoly.vertices[1] = k_scale * b2Vec2(.023,.033);
		LFootPoly.vertices[2] = RFootPoly.vertices[2] = k_scale * b2Vec2(.267,.035);
		LFootPoly.vertices[3] = RFootPoly.vertices[3] = k_scale * b2Vec2(.265,.065);
		LFootPoly.vertices[4] = RFootPoly.vertices[4] = k_scale * b2Vec2(.117,.143);
	}
	{	// calves
		LCalfPoly.vertexCount = RCalfPoly.vertexCount = 4;
		LCalfPoly.vertices[0] = RCalfPoly.vertices[0] = k_scale * b2Vec2(.089,.016);
		LCalfPoly.vertices[1] = RCalfPoly.vertices[1] = k_scale * b2Vec2(.178,.016);
		LCalfPoly.vertices[2] = RCalfPoly.vertices[2] = k_scale * b2Vec2(.205,.417);
		LCalfPoly.vertices[3] = RCalfPoly.vertices[3] = k_scale * b2Vec2(.095,.417);
	}
	{	// thighs
		LThighPoly.vertexCount = RThighPoly.vertexCount = 4;
		LThighPoly.vertices[0] = RThighPoly.vertices[0] = k_scale * b2Vec2(.137,.032);
		LThighPoly.vertices[1] = RThighPoly.vertices[1] = k_scale * b2Vec2(.243,.032);
		LThighPoly.vertices[2] = RThighPoly.vertices[2] = k_scale * b2Vec2(.318,.343);
		LThighPoly.vertices[3] = RThighPoly.vertices[3] = k_scale * b2Vec2(.142,.343);
	}
	{	// pelvis
		PelvisPoly.vertexCount = 5;
		PelvisPoly.vertices[0] = k_scale * b2Vec2(.105,.051);
		PelvisPoly.vertices[1] = k_scale * b2Vec2(.277,.053);
		PelvisPoly.vertices[2] = k_scale * b2Vec2(.320,.233);
		PelvisPoly.vertices[3] = k_scale * b2Vec2(.112,.233);
		PelvisPoly.vertices[4] = k_scale * b2Vec2(.067,.152);
	}
	{	// stomach
		StomachPoly.vertexCount = 4;
		StomachPoly.vertices[0] = k_scale * b2Vec2(.088,.043);
		StomachPoly.vertices[1] = k_scale * b2Vec2(.284,.043);
		StomachPoly.vertices[2] = k_scale * b2Vec2(.295,.231);
		StomachPoly.vertices[3] = k_scale * b2Vec2(.100,.231);
	}
	{	// chest
		ChestPoly.vertexCount = 4;
		ChestPoly.vertices[0] = k_scale * b2Vec2(.091,.042);
		ChestPoly.vertices[1] = k_scale * b2Vec2(.283,.042);
		ChestPoly.vertices[2] = k_scale * b2Vec2(.177,.289);
		ChestPoly.vertices[3] = k_scale * b2Vec2(.065,.289);
	}
	{	// head
		HeadCirc.radius = k_scale * .115;
	}
	{	// neck
		NeckPoly.vertexCount = 4;
		NeckPoly.vertices[0] = k_scale * b2Vec2(.038,.054);
		NeckPoly.vertices[1] = k_scale * b2Vec2(.149,.054);
		NeckPoly.vertices[2] = k_scale * b2Vec2(.154,.102);
		NeckPoly.vertices[3] = k_scale * b2Vec2(.054,.113);
	}
	{	// upper arms
		LUpperArmPoly.vertexCount = RUpperArmPoly.vertexCount = 5;
		LUpperArmPoly.vertices[0] = RUpperArmPoly.vertices[0] = k_scale * b2Vec2(.092,.059);
		LUpperArmPoly.vertices[1] = RUpperArmPoly.vertices[1] = k_scale * b2Vec2(.159,.059);
		LUpperArmPoly.vertices[2] = RUpperArmPoly.vertices[2] = k_scale * b2Vec2(.169,.335);
		LUpperArmPoly.vertices[3] = RUpperArmPoly.vertices[3] = k_scale * b2Vec2(.078,.335);
		LUpperArmPoly.vertices[4] = RUpperArmPoly.vertices[4] = k_scale * b2Vec2(.064,.248);
	}
	{	// forearms
		LForearmPoly.vertexCount = RForearmPoly.vertexCount = 4;
		LForearmPoly.vertices[0] = RForearmPoly.vertices[0] = k_scale * b2Vec2(.082,.054);
		LForearmPoly.vertices[1] = RForearmPoly.vertices[1] = k_scale * b2Vec2(.138,.054);
		LForearmPoly.vertices[2] = RForearmPoly.vertices[2] = k_scale * b2Vec2(.149,.296);
		LForearmPoly.vertices[3] = RForearmPoly.vertices[3] = k_scale * b2Vec2(.088,.296);
	}
	{	// hands
		LHandPoly.vertexCount = RHandPoly.vertexCount = 5;
		LHandPoly.vertices[0] = RHandPoly.vertices[0] = k_scale * b2Vec2(.066,.031);
		LHandPoly.vertices[1] = RHandPoly.vertices[1] = k_scale * b2Vec2(.123,.020);
		LHandPoly.vertices[2] = RHandPoly.vertices[2] = k_scale * b2Vec2(.160,.127);
		LHandPoly.vertices[3] = RHandPoly.vertices[3] = k_scale * b2Vec2(.127,.178);
		LHandPoly.vertices[4] = RHandPoly.vertices[4] = k_scale * b2Vec2(.074,.178);;
	}
}

void BipedDef::DefaultJoints()
{
	//b.LAnkleDef.body1		= LFoot;
	//b.LAnkleDef.body2		= LCalf;
	//b.RAnkleDef.body1		= RFoot;
	//b.RAnkleDef.body2		= RCalf;
	{	// ankles
		b2Vec2 anchor = k_scale * b2Vec2(-.045,-.75);
		LAnkleDef.localAnchor1		= RAnkleDef.localAnchor1	= anchor - LFootDef.position;
		LAnkleDef.localAnchor2		= RAnkleDef.localAnchor2	= anchor - LCalfDef.position;
		LAnkleDef.referenceAngle	= RAnkleDef.referenceAngle	= 0.0;
		LAnkleDef.lowerAngle		= RAnkleDef.lowerAngle		= -0.523598776;
		LAnkleDef.upperAngle		= RAnkleDef.upperAngle		= 0.523598776;
	}

	//b.LKneeDef.body1		= LCalf;
	//b.LKneeDef.body2		= LThigh;
	//b.RKneeDef.body1		= RCalf;
	//b.RKneeDef.body2		= RThigh;
	{	// knees
		b2Vec2 anchor = k_scale * b2Vec2(-.030,-.355);
		LKneeDef.localAnchor1	= RKneeDef.localAnchor1		= anchor - LCalfDef.position;
		LKneeDef.localAnchor2	= RKneeDef.localAnchor2		= anchor - LThighDef.position;
		LKneeDef.referenceAngle	= RKneeDef.referenceAngle	= 0.0;
		LKneeDef.lowerAngle		= RKneeDef.lowerAngle		= 0;
		LKneeDef.upperAngle		= RKneeDef.upperAngle		= 2.61799388;
	}

	//b.LHipDef.body1			= LThigh;
	//b.LHipDef.body2			= Pelvis;
	//b.RHipDef.body1			= RThigh;
	//b.RHipDef.body2			= Pelvis;
	{	// hips
		b2Vec2 anchor = k_scale * b2Vec2(.005,-.045);
		LHipDef.localAnchor1	= RHipDef.localAnchor1		= anchor - LThighDef.position;
		LHipDef.localAnchor2	= RHipDef.localAnchor2		= anchor - PelvisDef.position;
		LHipDef.referenceAngle	= RHipDef.referenceAngle	= 0.0;
		LHipDef.lowerAngle		= RHipDef.lowerAngle		= -2.26892803;
		LHipDef.upperAngle		= RHipDef.upperAngle		= 0;
	}

	//b.LowerAbsDef.body1		= Pelvis;
	//b.LowerAbsDef.body2		= Stomach;
	{	// lower abs
		b2Vec2 anchor = k_scale * b2Vec2(.035,.135);
		LowerAbsDef.localAnchor1	= anchor - PelvisDef.position;
		LowerAbsDef.localAnchor2	= anchor - StomachDef.position;
		LowerAbsDef.referenceAngle	= 0.0;
		LowerAbsDef.lowerAngle		= -0.523598776;
		LowerAbsDef.upperAngle		= 0.523598776;
	}

	//b.UpperAbsDef.body1		= Stomach;
	//b.UpperAbsDef.body2		= Chest;
	{	// upper abs
		b2Vec2 anchor = k_scale * b2Vec2(.045,.320);
		UpperAbsDef.localAnchor1	= anchor - StomachDef.position;
		UpperAbsDef.localAnchor2	= anchor - ChestDef.position;
		UpperAbsDef.referenceAngle	= 0.0;
		UpperAbsDef.lowerAngle		= -0.523598776;
		UpperAbsDef.upperAngle		= 0.174532925;
	}

	//b.LowerNeckDef.body1	= Chest;
	//b.LowerNeckDef.body2	= Neck;
	{	// lower neck
		b2Vec2 anchor = k_scale * b2Vec2(-.015,.575);
		LowerNeckDef.localAnchor1	= anchor - ChestDef.position;
		LowerNeckDef.localAnchor2	= anchor - NeckDef.position;
		LowerNeckDef.referenceAngle	= 0.0;
		LowerNeckDef.lowerAngle		= -0.174532925;
		LowerNeckDef.upperAngle		= 0.174532925;
	}

	//b.UpperNeckDef.body1	= Chest;
	//b.UpperNeckDef.body2	= Head;
	{	// upper neck
		b2Vec2 anchor = k_scale * b2Vec2(-.005,.630);
		UpperNeckDef.localAnchor1	= anchor - ChestDef.position;
		UpperNeckDef.localAnchor2	= anchor - HeadDef.position;
		UpperNeckDef.referenceAngle	= 0.0;
		UpperNeckDef.lowerAngle		= -0.610865238;
		UpperNeckDef.upperAngle		= 0.785398163;
	}

	//b.LShoulderDef.body1	= Chest;
	//b.LShoulderDef.body2	= LUpperArm;
	//b.RShoulderDef.body1	= Chest;
	//b.RShoulderDef.body2	= RUpperArm;
	{	// shoulders
		b2Vec2 anchor = k_scale * b2Vec2(-.015,.545);
		LShoulderDef.localAnchor1	= RShoulderDef.localAnchor1		= anchor - ChestDef.position;
		LShoulderDef.localAnchor2	= RShoulderDef.localAnchor2		= anchor - LUpperArmDef.position;
		LShoulderDef.referenceAngle	= RShoulderDef.referenceAngle	= 0.0;
		LShoulderDef.lowerAngle		= RShoulderDef.lowerAngle		= -1.04719755;
		LShoulderDef.upperAngle		= RShoulderDef.upperAngle		= 3.14159265;
	}

	//b.LElbowDef.body1		= LForearm;
	//b.LElbowDef.body2		= LUpperArm;
	//b.RElbowDef.body1		= RForearm;
	//b.RElbowDef.body2		= RUpperArm;
	{	// elbows
		b2Vec2 anchor = k_scale * b2Vec2(-.005,.290);
		LElbowDef.localAnchor1		= RElbowDef.localAnchor1	= anchor - LForearmDef.position;
		LElbowDef.localAnchor2		= RElbowDef.localAnchor2	= anchor - LUpperArmDef.position;
		LElbowDef.referenceAngle	= RElbowDef.referenceAngle	= 0.0;
		LElbowDef.lowerAngle		= RElbowDef.lowerAngle		= -2.7925268;
		LElbowDef.upperAngle		= RElbowDef.upperAngle		= 0;
	}

	//b.LWristDef.body1		= LHand;
	//b.LWristDef.body2		= LForearm;
	//b.RWristDef.body1		= RHand;
	//b.RWristDef.body2		= RForearm;
	{	// wrists
		b2Vec2 anchor = k_scale * b2Vec2(-.010,.045);
		LWristDef.localAnchor1		= RWristDef.localAnchor1	= anchor - LHandDef.position;
		LWristDef.localAnchor2		= RWristDef.localAnchor2	= anchor - LForearmDef.position;
		LWristDef.referenceAngle	= RWristDef.referenceAngle	= 0.0;
		LWristDef.lowerAngle		= RWristDef.lowerAngle		= -0.174532925;
		LWristDef.upperAngle		= RWristDef.upperAngle		= 0.174532925;
	}
}

void BipedDef::DefaultPositions()
{
	LFootDef.position		= RFootDef.position			= k_scale * b2Vec2(-.122,-.901);
	LCalfDef.position		= RCalfDef.position			= k_scale * b2Vec2(-.177,-.771);
	LThighDef.position		= RThighDef.position		= k_scale * b2Vec2(-.217,-.391);
	LUpperArmDef.position	= RUpperArmDef.position		= k_scale * b2Vec2(-.127,.228);
	LForearmDef.position	= RForearmDef.position		= k_scale * b2Vec2(-.117,-.011);
	LHandDef.position		= RHandDef.position			= k_scale * b2Vec2(-.112,-.136);
	PelvisDef.position									= k_scale * b2Vec2(-.177,-.101);
	StomachDef.position									= k_scale * b2Vec2(-.142,.088);
	ChestDef.position									= k_scale * b2Vec2(-.132,.282);
	NeckDef.position									= k_scale * b2Vec2(-.102,.518);
	HeadDef.position									= k_scale * b2Vec2(.022,.738);
}
