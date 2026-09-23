from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


class ACPUCacheConfigTest(unittest.TestCase):
    def test_acpu_shared_psram_uses_write_through_for_ipc_buffers(self):
        kconfig = (ROOT / "acpu/Kconfig.proj").read_text()
        self.assertNotIn("config PSRAM_CACHE_WB", kconfig)


if __name__ == "__main__":
    unittest.main()
