#include <jni.h>

JNIEXPORT void JNICALL
Java_edu_byu_rvl_myvisiondriveapp_MyVisionDriveActivity_FindFeatures(JNIEnv *env, jobject instance,
                                                                     jlong matAddrGr,
                                                                     jlong matAddrRgba) {

//   return (*env)->NewStringUTF(env, "Made it!");
   // TODO

}

JNIEXPORT jstring JNICALL
Java_edu_byu_rvl_myvisiondriveapp_MyVisionDriveActivity_getMsgFromJni(JNIEnv *env,
                                                                      jobject instance) {

   // TODO


   return (*env)->NewStringUTF(env, "made it!");
}