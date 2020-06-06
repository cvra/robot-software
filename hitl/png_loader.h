#pragma once

// Loads an OpenGL texture from the given path, and returns its OpenGL texture ID
int texture_load(const char* file, int* width, int* height);
