---
weight: 20
title: API Reference
---

# API

This section is meant to be a quick reference for Okapi's entire API, including methods the user may not normally interact with. This reference is broken into sections, covering one class per section. Subclasses are placed below the base class, but in different sections. If a class is marked (abstract) then it contains one or more pure virtual functions, and cannot be instantiated (it is designed only to be an interface).

<aside class="notice">
Remember that derived classes inherit the interface of their base class; therefore, derived classes will not have their base class' functions documented (you can safely assume that all functions from the base class are implemented).
</aside>
