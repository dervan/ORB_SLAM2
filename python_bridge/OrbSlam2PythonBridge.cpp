#include <Python.h>
#include <numpy/ndarraytypes.h>
#include <numpy/arrayobject.h>
#include"System.h"

static ORB_SLAM2::System * slam;

static PyObject *
odom_track(PyObject *self, PyObject *args)
{

  PyArrayObject *imageArr;  // The python objects to be extracted from the args
  unsigned char * image;
  double time;
  /* Parse tuples separately since args will differ between C fcns */
  if (!PyArg_ParseTuple(args, "Od", &imageArr, &time) )
    return NULL;
  int rows = PyArray_SHAPE(imageArr)[0];
  int cols = PyArray_SHAPE(imageArr)[1];
  image = static_cast<unsigned char*>(PyArray_DATA(imageArr));
  cv::Mat cv_image(rows, cols, CV_8U, image);

  cv::Mat pose = slam->TrackMonocular(cv_image, time);
  
  if(pose.rows != 0){
    npy_intp pose_size[] = {pose.rows, pose.cols};
    float* data = new float[pose.rows*pose.cols];
    std::memcpy(data, pose.data, pose.rows*pose.cols*sizeof(float));
    PyObject * np_pose = PyArray_SimpleNewFromData(2, pose_size, NPY_FLOAT, (void*)data);
    PyArray_ENABLEFLAGS((PyArrayObject*)np_pose, NPY_ARRAY_OWNDATA);
    //Py_INCREF(np_pose);
    return np_pose;
  } else {
    Py_RETURN_NONE;
  }
}

static PyObject *
odom_init(PyObject *self, PyObject *args)
{
  char * vocabulary;
  char * config_yaml;
  if (!PyArg_ParseTuple(args, "ss", &vocabulary, &config_yaml))
    return NULL;

  slam = new ORB_SLAM2::System(vocabulary, config_yaml, ORB_SLAM2::System::MONOCULAR);
  Py_RETURN_NONE;
}


static PyObject *
odom_reset(PyObject *self, PyObject *args)
{
  slam->Reset();
  Py_RETURN_NONE;
}

static PyMethodDef OdomMethods[] = {
  {"track", odom_track, METH_VARARGS, "Track new photo in SLAM"},
  {"init", odom_init, METH_VARARGS, "Initialize SLAM"},
  {"reset", odom_reset, METH_NOARGS, "Resets SLAM"},
  {NULL, NULL, 0, NULL}
};

static struct PyModuleDef odommodule = {
  PyModuleDef_HEAD_INIT,
  "odom",
  NULL, // Documentation
  -1,
  OdomMethods
};

PyMODINIT_FUNC
PyInit_orb_slam(void){
  import_array();
  return PyModule_Create(&odommodule);
}

