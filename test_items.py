import pytest
from selenium import webdriver
from selenium.webdriver.common.by import By
from selenium.webdriver.support.ui import WebDriverWait
from selenium.webdriver.support import expected_conditions as EC
import time


class TestLanguages:

    def test_presence_of_add_button(self, browser):
        browser.get('http://selenium1py.pythonanywhere.com/catalogue/coders-at-work_207/')
        time.sleep(30)
        assert WebDriverWait(browser, 10).until(
            EC.visibility_of_element_located((By.CSS_SELECTOR, ".btn-add-to-basket"))
        ), "'add to basket' button isn't displayed"
