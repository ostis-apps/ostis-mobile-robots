#pragma once

#include <sc-memory/sc_module.hpp>

class TransportModule : public ScModule
{
public:
  void Initialize(ScMemoryContext * context);

  void Shutdown(ScMemoryContext * context);
};
