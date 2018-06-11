# User guide

Build using mdbook

```
mdbook build
```

This will generate a folder `book`.
Its content is copied to the `gh-pages` branch to generate the [online documentation](cvra.github.io/robot-software).

```
git checkout --orphan gh-pages
git add book/* -f
git mv book/* ../..
git commit
```
