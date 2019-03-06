# DO NOT EDIT
# This makefile makes sure all linkable targets are
# up-to-date with anything they link to
default:
	echo "Do not invoke directly"

# Rules to remove targets that are older than anything to which they
# link.  This forces Xcode to relink the targets from scratch.  It
# does not seem to check these dependencies itself.
PostBuild.Practical2_bin.Debug:
PostBuild.igl.Debug: /Users/amirvaxman/INFOMGP-Practical2/build/Debug/Practical2_bin
PostBuild.igl_opengl_glfw.Debug: /Users/amirvaxman/INFOMGP-Practical2/build/Debug/Practical2_bin
PostBuild.igl_opengl_glfw_imgui.Debug: /Users/amirvaxman/INFOMGP-Practical2/build/Debug/Practical2_bin
PostBuild.igl_opengl_glfw.Debug: /Users/amirvaxman/INFOMGP-Practical2/build/Debug/Practical2_bin
PostBuild.igl_opengl.Debug: /Users/amirvaxman/INFOMGP-Practical2/build/Debug/Practical2_bin
PostBuild.igl.Debug: /Users/amirvaxman/INFOMGP-Practical2/build/Debug/Practical2_bin
PostBuild.igl_common.Debug: /Users/amirvaxman/INFOMGP-Practical2/build/Debug/Practical2_bin
PostBuild.imgui.Debug: /Users/amirvaxman/INFOMGP-Practical2/build/Debug/Practical2_bin
PostBuild.glfw.Debug: /Users/amirvaxman/INFOMGP-Practical2/build/Debug/Practical2_bin
PostBuild.glad.Debug: /Users/amirvaxman/INFOMGP-Practical2/build/Debug/Practical2_bin
/Users/amirvaxman/INFOMGP-Practical2/build/Debug/Practical2_bin:\
	/Users/amirvaxman/INFOMGP-Practical2/build/imgui/Debug/libimgui.a\
	/Users/amirvaxman/INFOMGP-Practical2/build/glfw/src/Debug/libglfw3.a\
	/Users/amirvaxman/INFOMGP-Practical2/build/glad/Debug/libglad.a
	/bin/rm -f /Users/amirvaxman/INFOMGP-Practical2/build/Debug/Practical2_bin


PostBuild.glad.Debug:
/Users/amirvaxman/INFOMGP-Practical2/build/glad/Debug/libglad.a:
	/bin/rm -f /Users/amirvaxman/INFOMGP-Practical2/build/glad/Debug/libglad.a


PostBuild.glfw.Debug:
/Users/amirvaxman/INFOMGP-Practical2/build/glfw/src/Debug/libglfw3.a:
	/bin/rm -f /Users/amirvaxman/INFOMGP-Practical2/build/glfw/src/Debug/libglfw3.a


PostBuild.imgui.Debug:
/Users/amirvaxman/INFOMGP-Practical2/build/imgui/Debug/libimgui.a:
	/bin/rm -f /Users/amirvaxman/INFOMGP-Practical2/build/imgui/Debug/libimgui.a


PostBuild.Practical2_bin.Release:
PostBuild.igl.Release: /Users/amirvaxman/INFOMGP-Practical2/build/Release/Practical2_bin
PostBuild.igl_opengl_glfw.Release: /Users/amirvaxman/INFOMGP-Practical2/build/Release/Practical2_bin
PostBuild.igl_opengl_glfw_imgui.Release: /Users/amirvaxman/INFOMGP-Practical2/build/Release/Practical2_bin
PostBuild.igl_opengl_glfw.Release: /Users/amirvaxman/INFOMGP-Practical2/build/Release/Practical2_bin
PostBuild.igl_opengl.Release: /Users/amirvaxman/INFOMGP-Practical2/build/Release/Practical2_bin
PostBuild.igl.Release: /Users/amirvaxman/INFOMGP-Practical2/build/Release/Practical2_bin
PostBuild.igl_common.Release: /Users/amirvaxman/INFOMGP-Practical2/build/Release/Practical2_bin
PostBuild.imgui.Release: /Users/amirvaxman/INFOMGP-Practical2/build/Release/Practical2_bin
PostBuild.glfw.Release: /Users/amirvaxman/INFOMGP-Practical2/build/Release/Practical2_bin
PostBuild.glad.Release: /Users/amirvaxman/INFOMGP-Practical2/build/Release/Practical2_bin
/Users/amirvaxman/INFOMGP-Practical2/build/Release/Practical2_bin:\
	/Users/amirvaxman/INFOMGP-Practical2/build/imgui/Release/libimgui.a\
	/Users/amirvaxman/INFOMGP-Practical2/build/glfw/src/Release/libglfw3.a\
	/Users/amirvaxman/INFOMGP-Practical2/build/glad/Release/libglad.a
	/bin/rm -f /Users/amirvaxman/INFOMGP-Practical2/build/Release/Practical2_bin


PostBuild.glad.Release:
/Users/amirvaxman/INFOMGP-Practical2/build/glad/Release/libglad.a:
	/bin/rm -f /Users/amirvaxman/INFOMGP-Practical2/build/glad/Release/libglad.a


PostBuild.glfw.Release:
/Users/amirvaxman/INFOMGP-Practical2/build/glfw/src/Release/libglfw3.a:
	/bin/rm -f /Users/amirvaxman/INFOMGP-Practical2/build/glfw/src/Release/libglfw3.a


PostBuild.imgui.Release:
/Users/amirvaxman/INFOMGP-Practical2/build/imgui/Release/libimgui.a:
	/bin/rm -f /Users/amirvaxman/INFOMGP-Practical2/build/imgui/Release/libimgui.a


PostBuild.Practical2_bin.MinSizeRel:
PostBuild.igl.MinSizeRel: /Users/amirvaxman/INFOMGP-Practical2/build/MinSizeRel/Practical2_bin
PostBuild.igl_opengl_glfw.MinSizeRel: /Users/amirvaxman/INFOMGP-Practical2/build/MinSizeRel/Practical2_bin
PostBuild.igl_opengl_glfw_imgui.MinSizeRel: /Users/amirvaxman/INFOMGP-Practical2/build/MinSizeRel/Practical2_bin
PostBuild.igl_opengl_glfw.MinSizeRel: /Users/amirvaxman/INFOMGP-Practical2/build/MinSizeRel/Practical2_bin
PostBuild.igl_opengl.MinSizeRel: /Users/amirvaxman/INFOMGP-Practical2/build/MinSizeRel/Practical2_bin
PostBuild.igl.MinSizeRel: /Users/amirvaxman/INFOMGP-Practical2/build/MinSizeRel/Practical2_bin
PostBuild.igl_common.MinSizeRel: /Users/amirvaxman/INFOMGP-Practical2/build/MinSizeRel/Practical2_bin
PostBuild.imgui.MinSizeRel: /Users/amirvaxman/INFOMGP-Practical2/build/MinSizeRel/Practical2_bin
PostBuild.glfw.MinSizeRel: /Users/amirvaxman/INFOMGP-Practical2/build/MinSizeRel/Practical2_bin
PostBuild.glad.MinSizeRel: /Users/amirvaxman/INFOMGP-Practical2/build/MinSizeRel/Practical2_bin
/Users/amirvaxman/INFOMGP-Practical2/build/MinSizeRel/Practical2_bin:\
	/Users/amirvaxman/INFOMGP-Practical2/build/imgui/MinSizeRel/libimgui.a\
	/Users/amirvaxman/INFOMGP-Practical2/build/glfw/src/MinSizeRel/libglfw3.a\
	/Users/amirvaxman/INFOMGP-Practical2/build/glad/MinSizeRel/libglad.a
	/bin/rm -f /Users/amirvaxman/INFOMGP-Practical2/build/MinSizeRel/Practical2_bin


PostBuild.glad.MinSizeRel:
/Users/amirvaxman/INFOMGP-Practical2/build/glad/MinSizeRel/libglad.a:
	/bin/rm -f /Users/amirvaxman/INFOMGP-Practical2/build/glad/MinSizeRel/libglad.a


PostBuild.glfw.MinSizeRel:
/Users/amirvaxman/INFOMGP-Practical2/build/glfw/src/MinSizeRel/libglfw3.a:
	/bin/rm -f /Users/amirvaxman/INFOMGP-Practical2/build/glfw/src/MinSizeRel/libglfw3.a


PostBuild.imgui.MinSizeRel:
/Users/amirvaxman/INFOMGP-Practical2/build/imgui/MinSizeRel/libimgui.a:
	/bin/rm -f /Users/amirvaxman/INFOMGP-Practical2/build/imgui/MinSizeRel/libimgui.a


PostBuild.Practical2_bin.RelWithDebInfo:
PostBuild.igl.RelWithDebInfo: /Users/amirvaxman/INFOMGP-Practical2/build/RelWithDebInfo/Practical2_bin
PostBuild.igl_opengl_glfw.RelWithDebInfo: /Users/amirvaxman/INFOMGP-Practical2/build/RelWithDebInfo/Practical2_bin
PostBuild.igl_opengl_glfw_imgui.RelWithDebInfo: /Users/amirvaxman/INFOMGP-Practical2/build/RelWithDebInfo/Practical2_bin
PostBuild.igl_opengl_glfw.RelWithDebInfo: /Users/amirvaxman/INFOMGP-Practical2/build/RelWithDebInfo/Practical2_bin
PostBuild.igl_opengl.RelWithDebInfo: /Users/amirvaxman/INFOMGP-Practical2/build/RelWithDebInfo/Practical2_bin
PostBuild.igl.RelWithDebInfo: /Users/amirvaxman/INFOMGP-Practical2/build/RelWithDebInfo/Practical2_bin
PostBuild.igl_common.RelWithDebInfo: /Users/amirvaxman/INFOMGP-Practical2/build/RelWithDebInfo/Practical2_bin
PostBuild.imgui.RelWithDebInfo: /Users/amirvaxman/INFOMGP-Practical2/build/RelWithDebInfo/Practical2_bin
PostBuild.glfw.RelWithDebInfo: /Users/amirvaxman/INFOMGP-Practical2/build/RelWithDebInfo/Practical2_bin
PostBuild.glad.RelWithDebInfo: /Users/amirvaxman/INFOMGP-Practical2/build/RelWithDebInfo/Practical2_bin
/Users/amirvaxman/INFOMGP-Practical2/build/RelWithDebInfo/Practical2_bin:\
	/Users/amirvaxman/INFOMGP-Practical2/build/imgui/RelWithDebInfo/libimgui.a\
	/Users/amirvaxman/INFOMGP-Practical2/build/glfw/src/RelWithDebInfo/libglfw3.a\
	/Users/amirvaxman/INFOMGP-Practical2/build/glad/RelWithDebInfo/libglad.a
	/bin/rm -f /Users/amirvaxman/INFOMGP-Practical2/build/RelWithDebInfo/Practical2_bin


PostBuild.glad.RelWithDebInfo:
/Users/amirvaxman/INFOMGP-Practical2/build/glad/RelWithDebInfo/libglad.a:
	/bin/rm -f /Users/amirvaxman/INFOMGP-Practical2/build/glad/RelWithDebInfo/libglad.a


PostBuild.glfw.RelWithDebInfo:
/Users/amirvaxman/INFOMGP-Practical2/build/glfw/src/RelWithDebInfo/libglfw3.a:
	/bin/rm -f /Users/amirvaxman/INFOMGP-Practical2/build/glfw/src/RelWithDebInfo/libglfw3.a


PostBuild.imgui.RelWithDebInfo:
/Users/amirvaxman/INFOMGP-Practical2/build/imgui/RelWithDebInfo/libimgui.a:
	/bin/rm -f /Users/amirvaxman/INFOMGP-Practical2/build/imgui/RelWithDebInfo/libimgui.a




# For each target create a dummy ruleso the target does not have to exist
/Users/amirvaxman/INFOMGP-Practical2/build/glad/Debug/libglad.a:
/Users/amirvaxman/INFOMGP-Practical2/build/glad/MinSizeRel/libglad.a:
/Users/amirvaxman/INFOMGP-Practical2/build/glad/RelWithDebInfo/libglad.a:
/Users/amirvaxman/INFOMGP-Practical2/build/glad/Release/libglad.a:
/Users/amirvaxman/INFOMGP-Practical2/build/glfw/src/Debug/libglfw3.a:
/Users/amirvaxman/INFOMGP-Practical2/build/glfw/src/MinSizeRel/libglfw3.a:
/Users/amirvaxman/INFOMGP-Practical2/build/glfw/src/RelWithDebInfo/libglfw3.a:
/Users/amirvaxman/INFOMGP-Practical2/build/glfw/src/Release/libglfw3.a:
/Users/amirvaxman/INFOMGP-Practical2/build/imgui/Debug/libimgui.a:
/Users/amirvaxman/INFOMGP-Practical2/build/imgui/MinSizeRel/libimgui.a:
/Users/amirvaxman/INFOMGP-Practical2/build/imgui/RelWithDebInfo/libimgui.a:
/Users/amirvaxman/INFOMGP-Practical2/build/imgui/Release/libimgui.a:
