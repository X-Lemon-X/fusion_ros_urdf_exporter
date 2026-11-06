"""Fake adsk.fusion: only the enums and cast the exporter touches."""


class MeshRefinementSettings:
    MeshRefinementHigh = 0
    MeshRefinementLow = 1
    MeshRefinementMedium = 2


class CalculationAccuracy:
    MediumCalculationAccuracy = 1


class Design:
    @staticmethod
    def cast(obj):
        return obj
