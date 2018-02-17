#include <Python.h>
#include "dobby_pwm.h"

static char module_docstring[] =
      "Module to handle pwm outputs to motors via PRUSS 1"

static char init_pwm_pru_docstring[] =
      "Initialise the pru for pwm"

static char stop_pwm_pru_docstring[] =
      "Exit the pru"

static char exec_dobby_pwm_docstring[] =
      "Load PRU firmware to PRUSS 1"

static char write_pwm_pulsewidth_docstring[] =
      "Write PWM pulsewidth in microseconds"

static char convert_pwm_pulsewidth_docstring[] =
      "Convert pwm pulsewidth to value to be passed to PRU"

static PyObject *dobby_pwm_init_pwm_pru(PyObject *self, PyObject *args);
static PyObject *dobby_pwm_stop_pwm_pru(PyObject *self, PyObject *args);
static PyObject *dobby_pwm_exec_dobby_pwm(PyObject *self, PyObject *args);
static PyObject *dobby_pwm_write_pwm_pulsewidth(PyObject *self, PyObject *args);
static PyObject *dobby_pwm_convert_pwm_pulsewidth(PyObject *self, PyObject *args);

static PyMethodDef module_methods[] = {
    {"init_pwm_pru", dobby_pwm_init_pwm_pru, METH_VARARGS, init_pwm_pru_docstring},
    {"stop_pwm_pru", dobby_pwm_stop_pwm_pru, METH_VARARGS, stop_pwm_pru_docstring},
    {"exec_dobby_pwm", dobby_pwm_exec_dobby_pwm, METH_VARARGS, exec_dobby_pwm_docstring},
    {"write_pwm_pulsewidth", dobby_pwm_write_pwm_pulsewidth, METH_VARARGS, write_pwm_pulsewidth_docstring},
    {"convert_pwm_pulsewidth", dobby_pwm_convert_pwm_pulsewidth, METH_VARARGS, convert_pwm_pulsewidth_docstring},
    {NULL, NULL, 0, NULL}
};

PyMODINIT_FUNC init_dobby_pwm(void) {
    PyObject *m = Py_InitModule3("_dobby_pwm", module_methods, module_docstring);
    if (m == NULL)
        return;
}
