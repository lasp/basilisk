# run as `env PYTHONPATH=build/dist/ python3 test.py`
import basilisk as bsk
import mrp_steering as mrp

m = mrp.DemoModule()

print(m)
print("Inputs:", m.inputs)
print("Outputs:", m.outputs)

m.subscribeTo({
  "rate": m.outputs["foo"]["y"],
  "foo": m.outputs["foo"],
})

bsk.Fanout([m.inputs["foo"], m.inputs["foo"]]).subscribeTo(m.outputs["foo"])
