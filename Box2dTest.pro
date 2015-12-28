TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt
CONFIG += c++11

QMAKE_CXXFLAGS += -std=c++11 -pthread
QMAKE_CFLAGS += -std=c11 -pthread

QMAKE_CFLAGS_DEBUG += -O0 -pthread
QMAKE_CXXFLAGS_DEBUG += -O0 -pthread

DEFINES += _GLFW_X11
DEFINES += _GLFW_GLX
DEFINES += _GLFW_EGL_NATIVE_WINDOW

INCLUDEPATH += $$PWD/Box2D/
INCLUDEPATH += $$PWD/Box2D/Testbed/glfw/include/GLFW/
INCLUDEPATH += $$PWD/Box2D/Testbed/imgui/
INCLUDEPATH += $$PWD/Box2D/Testbed/glfw/deps/

LIBS += -L/usr/lib -pthread -ldl -lX11 -lXxf86vm -lXrandr -lXi -lXinerama -lXcursor

HEADERS += \
    Box2D/Box2D/Collision/Shapes/b2ChainShape.h \
    Box2D/Box2D/Collision/Shapes/b2CircleShape.h \
    Box2D/Box2D/Collision/Shapes/b2EdgeShape.h \
    Box2D/Box2D/Collision/Shapes/b2PolygonShape.h \
    Box2D/Box2D/Collision/Shapes/b2Shape.h \
    Box2D/Box2D/Collision/b2BroadPhase.h \
    Box2D/Box2D/Collision/b2Collision.h \
    Box2D/Box2D/Collision/b2Distance.h \
    Box2D/Box2D/Collision/b2DynamicTree.h \
    Box2D/Box2D/Collision/b2TimeOfImpact.h \
    Box2D/Box2D/Common/b2BlockAllocator.h \
    Box2D/Box2D/Common/b2Draw.h \
    Box2D/Box2D/Common/b2GrowableStack.h \
    Box2D/Box2D/Common/b2Math.h \
    Box2D/Box2D/Common/b2Settings.h \
    Box2D/Box2D/Common/b2StackAllocator.h \
    Box2D/Box2D/Common/b2Timer.h \
    Box2D/Box2D/Dynamics/Contacts/b2ChainAndCircleContact.h \
    Box2D/Box2D/Dynamics/Contacts/b2ChainAndPolygonContact.h \
    Box2D/Box2D/Dynamics/Contacts/b2CircleContact.h \
    Box2D/Box2D/Dynamics/Contacts/b2Contact.h \
    Box2D/Box2D/Dynamics/Contacts/b2ContactSolver.h \
    Box2D/Box2D/Dynamics/Contacts/b2EdgeAndCircleContact.h \
    Box2D/Box2D/Dynamics/Contacts/b2EdgeAndPolygonContact.h \
    Box2D/Box2D/Dynamics/Contacts/b2PolygonAndCircleContact.h \
    Box2D/Box2D/Dynamics/Contacts/b2PolygonContact.h \
    Box2D/Box2D/Dynamics/Joints/b2DistanceJoint.h \
    Box2D/Box2D/Dynamics/Joints/b2FrictionJoint.h \
    Box2D/Box2D/Dynamics/Joints/b2GearJoint.h \
    Box2D/Box2D/Dynamics/Joints/b2Joint.h \
    Box2D/Box2D/Dynamics/Joints/b2MotorJoint.h \
    Box2D/Box2D/Dynamics/Joints/b2MouseJoint.h \
    Box2D/Box2D/Dynamics/Joints/b2PrismaticJoint.h \
    Box2D/Box2D/Dynamics/Joints/b2PulleyJoint.h \
    Box2D/Box2D/Dynamics/Joints/b2RevoluteJoint.h \
    Box2D/Box2D/Dynamics/Joints/b2RopeJoint.h \
    Box2D/Box2D/Dynamics/Joints/b2WeldJoint.h \
    Box2D/Box2D/Dynamics/Joints/b2WheelJoint.h \
    Box2D/Box2D/Dynamics/b2Body.h \
    Box2D/Box2D/Dynamics/b2ContactManager.h \
    Box2D/Box2D/Dynamics/b2Fixture.h \
    Box2D/Box2D/Dynamics/b2Island.h \
    Box2D/Box2D/Dynamics/b2TimeStep.h \
    Box2D/Box2D/Dynamics/b2World.h \
    Box2D/Box2D/Dynamics/b2WorldCallbacks.h \
    Box2D/Box2D/Rope/b2Rope.h \
    Box2D/Box2D/Box2D.h \
    Box2D/Testbed/Framework/DebugDraw.h \
    Box2D/Testbed/Framework/stb_truetype.h \
    Box2D/Testbed/Framework/Test.h \
    Box2D/Testbed/glfw/deps/glad/glad.h \
    Box2D/Testbed/glfw/deps/KHR/khrplatform.h \
    Box2D/Testbed/glfw/deps/getopt.h \
    Box2D/Testbed/glfw/deps/linmath.h \
    Box2D/Testbed/glfw/deps/tinycthread.h \
    Box2D/Testbed/glfw/include/GLFW/glfw3.h \
    Box2D/Testbed/glfw/include/GLFW/glfw3native.h \
    Box2D/Testbed/glfw/src/cocoa_joystick.h \
    Box2D/Testbed/glfw/src/cocoa_platform.h \
    Box2D/Testbed/glfw/src/egl_context.h \
    Box2D/Testbed/glfw/src/glx_context.h \
    Box2D/Testbed/glfw/src/internal.h \
    Box2D/Testbed/glfw/src/linux_joystick.h \
    Box2D/Testbed/glfw/src/mir_platform.h \
    Box2D/Testbed/glfw/src/nsgl_context.h \
    Box2D/Testbed/glfw/src/posix_time.h \
    Box2D/Testbed/glfw/src/posix_tls.h \
    Box2D/Testbed/glfw/src/wgl_context.h \
    Box2D/Testbed/glfw/src/win32_joystick.h \
    Box2D/Testbed/glfw/src/win32_platform.h \
    Box2D/Testbed/glfw/src/wl_platform.h \
    Box2D/Testbed/glfw/src/x11_platform.h \
    Box2D/Testbed/glfw/src/xkb_unicode.h \
    Box2D/Testbed/imgui/imconfig.h \
    Box2D/Testbed/imgui/imgui.h \
    Box2D/Testbed/imgui/imgui_internal.h \
    Box2D/Testbed/imgui/stb_rect_pack.h \
    Box2D/Testbed/imgui/stb_textedit.h \
    Box2D/Testbed/imgui/stb_truetype.h

SOURCES += \
    Box2D/Box2D/Collision/Shapes/b2ChainShape.cpp \
    Box2D/Box2D/Collision/Shapes/b2CircleShape.cpp \
    Box2D/Box2D/Collision/Shapes/b2EdgeShape.cpp \
    Box2D/Box2D/Collision/Shapes/b2PolygonShape.cpp \
    Box2D/Box2D/Collision/b2BroadPhase.cpp \
    Box2D/Box2D/Collision/b2CollideCircle.cpp \
    Box2D/Box2D/Collision/b2CollideEdge.cpp \
    Box2D/Box2D/Collision/b2CollidePolygon.cpp \
    Box2D/Box2D/Collision/b2Collision.cpp \
    Box2D/Box2D/Collision/b2Distance.cpp \
    Box2D/Box2D/Collision/b2DynamicTree.cpp \
    Box2D/Box2D/Collision/b2TimeOfImpact.cpp \
    Box2D/Box2D/Common/b2BlockAllocator.cpp \
    Box2D/Box2D/Common/b2Draw.cpp \
    Box2D/Box2D/Common/b2Math.cpp \
    Box2D/Box2D/Common/b2Settings.cpp \
    Box2D/Box2D/Common/b2StackAllocator.cpp \
    Box2D/Box2D/Common/b2Timer.cpp \
    Box2D/Box2D/Dynamics/Contacts/b2ChainAndCircleContact.cpp \
    Box2D/Box2D/Dynamics/Contacts/b2ChainAndPolygonContact.cpp \
    Box2D/Box2D/Dynamics/Contacts/b2CircleContact.cpp \
    Box2D/Box2D/Dynamics/Contacts/b2Contact.cpp \
    Box2D/Box2D/Dynamics/Contacts/b2ContactSolver.cpp \
    Box2D/Box2D/Dynamics/Contacts/b2EdgeAndCircleContact.cpp \
    Box2D/Box2D/Dynamics/Contacts/b2EdgeAndPolygonContact.cpp \
    Box2D/Box2D/Dynamics/Contacts/b2PolygonAndCircleContact.cpp \
    Box2D/Box2D/Dynamics/Contacts/b2PolygonContact.cpp \
    Box2D/Box2D/Dynamics/Joints/b2DistanceJoint.cpp \
    Box2D/Box2D/Dynamics/Joints/b2FrictionJoint.cpp \
    Box2D/Box2D/Dynamics/Joints/b2GearJoint.cpp \
    Box2D/Box2D/Dynamics/Joints/b2Joint.cpp \
    Box2D/Box2D/Dynamics/Joints/b2MotorJoint.cpp \
    Box2D/Box2D/Dynamics/Joints/b2MouseJoint.cpp \
    Box2D/Box2D/Dynamics/Joints/b2PrismaticJoint.cpp \
    Box2D/Box2D/Dynamics/Joints/b2PulleyJoint.cpp \
    Box2D/Box2D/Dynamics/Joints/b2RevoluteJoint.cpp \
    Box2D/Box2D/Dynamics/Joints/b2RopeJoint.cpp \
    Box2D/Box2D/Dynamics/Joints/b2WeldJoint.cpp \
    Box2D/Box2D/Dynamics/Joints/b2WheelJoint.cpp \
    Box2D/Box2D/Dynamics/b2Body.cpp \
    Box2D/Box2D/Dynamics/b2ContactManager.cpp \
    Box2D/Box2D/Dynamics/b2Fixture.cpp \
    Box2D/Box2D/Dynamics/b2Island.cpp \
    Box2D/Box2D/Dynamics/b2World.cpp \
    Box2D/Box2D/Dynamics/b2WorldCallbacks.cpp \
    Box2D/Box2D/Rope/b2Rope.cpp \
    Box2D/Testbed/Framework/DebugDraw.cpp \
    Box2D/Testbed/Framework/Main.cpp \
    Box2D/Testbed/Framework/Test.cpp \
    Box2D/Testbed/imgui/imgui.cpp \
    Box2D/Testbed/imgui/imgui_demo.cpp \
    Box2D/Testbed/imgui/imgui_draw.cpp \
    Box2D/Testbed/glfw/deps/getopt.c \
    Box2D/Testbed/glfw/deps/glad.c \
    Box2D/Testbed/glfw/deps/tinycthread.c \
    Box2D/Testbed/glfw/src/context.c \
    Box2D/Testbed/glfw/src/glx_context.c \
    Box2D/Testbed/glfw/src/init.c \
    Box2D/Testbed/glfw/src/input.c \
    Box2D/Testbed/glfw/src/linux_joystick.c \
    Box2D/Testbed/glfw/src/monitor.c \
    Box2D/Testbed/glfw/src/posix_time.c \
    Box2D/Testbed/glfw/src/posix_tls.c \
    Box2D/Testbed/glfw/src/window.c \
    Box2D/Testbed/glfw/src/x11_init.c \
    Box2D/Testbed/glfw/src/x11_monitor.c \
    Box2D/Testbed/glfw/src/x11_window.c \
    Box2D/Testbed/glfw/src/xkb_unicode.c \
    Box2D/Testbed/Tests/TestEntries.cpp
