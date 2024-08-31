# run as `env PYTHONPATH=build/dist/ python3 test.py`
import mrp_steering

m = mrp_steering.FooModule()

print(m)
print("Inputs:", m.inputs)
print("Outputs:", m.outputs)

m.subscribeTo({
  "rate": m.outputs["foo"]["y"],
  "foo": m.outputs["foo"],
})
