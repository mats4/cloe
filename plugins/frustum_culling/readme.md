# How to build

Go to root of cloe repository.

Execute:

```
make export-all
make -C meta package CONAN_OPTIONS="--build"
```

Then cd into plugins/frustum_culling
and 
``` make export package ```

If you want a more verbose output on the test cases, then run: \
``` make export package CONAN_OPTIONS="-o test_verbose=True" ```


# How to debug test cases

From plugins/frustum_culling, run:

``` conan install . -o test_verbose=True ```

``` conan build . ```

Then hit the 

