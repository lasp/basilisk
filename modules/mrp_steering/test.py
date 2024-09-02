# run as `env PYTHONPATH=build/dist/ python3 test.py`
import mrp_steering as mrp

m = mrp.DemoModule()

print(m)
print("Inputs:", m.inputs)
print("Outputs:", m.outputs)

m.subscribeTo({
  "rate": m.outputs["foo"]["y"],
  "foo": m.outputs["foo"],
})
