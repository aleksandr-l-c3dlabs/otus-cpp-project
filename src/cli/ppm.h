#pragma once

#include <ostream>

#include "framebuffer.h"

namespace rtr {

void ppm_export(std::ostream& stream, const FrameBuffer& buffer);

}